"""阿里云 Qwen 大模型 + function call，理解用户意图并解析成机器人动作指令。

模型 qwen-plus（质量/速度平衡，可换 qwen-turbo 更快更省 / qwen-max 更强）。
环境变量 DASHSCOPE_API_KEY 必须配置。

文档：https://help.aliyun.com/zh/model-studio/use-qwen-by-calling-api
"""
import os
import json
import threading
from http import HTTPStatus

import dashscope
from dashscope import Generation

dashscope.api_key = os.getenv('DASHSCOPE_API_KEY')

# 机器人动作工具定义（LLM function call）
TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "walk",
            "description": "控制机器人机体移动。必须处于行走类状态(rl_basic/rl_amp/rl_terrain)才能生效。",
            "parameters": {
                "type": "object",
                "properties": {
                    "linear_x": {
                        "type": "number",
                        "description": "前进速度 m/s，正前负后。慢走0.08~0.12，正常0.15，范围 -0.15~0.15"},
                    "linear_y": {
                        "type": "number",
                        "description": "左右平移 m/s，正向左负向右，范围 -0.10~0.10"},
                    "yaw": {
                        "type": "number",
                        "description": "偏航角速度 rad/s，正逆时针负顺时针，慢转0.1~0.2，范围 -0.30~0.30"}
                },
                "required": ["linear_x", "linear_y", "yaw"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "switch_motion",
            "description": "切换机器人运动状态。可选: idle, passive, pd_stand, rl_basic, rl_amp, rl_terrain, lower_body_balance, joint_bridge",
            "parameters": {
                "type": "object",
                "properties": {
                    "state": {
                        "type": "string",
                        "enum": ["idle", "passive", "pd_stand", "rl_basic",
                                 "rl_amp", "rl_terrain", "lower_body_balance",
                                 "joint_bridge"]
                    }
                },
                "required": ["state"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "wave_hand",
            "description": "挥手致意。需先 switch_motion 到 lower_body_balance 状态。",
            "parameters": {
                "type": "object",
                "properties": {
                    "side": {"type": "string", "enum": ["left", "right", "both"]}
                },
                "required": ["side"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "gesture",
            "description": "交警手势动作（模拟交警指挥，手臂动作）。手势ID: 0=归位, 1=停止, 2=直行, 3=左转弯, 4=左转弯待转, 5=右转弯, 6=变道, 7=减速慢行, 8=靠边停车。",
            "parameters": {
                "type": "object",
                "properties": {
                    "gesture_id": {
                        "type": "integer",
                        "description": "手势ID: 0=归位, 1=停止, 2=直行, 3=左转弯, 4=左转弯待转, 5=右转弯, 6=变道, 7=减速慢行, 8=靠边停车",
                        "enum": [0, 1, 2, 3, 4, 5, 6, 7, 8]
                    }
                },
                "required": ["gesture_id"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "emergency_stop",
            "description": "紧急停止，立即切到 passive 阻尼态。任何时候用户喊停或异常都用这个。",
            "parameters": {"type": "object", "properties": {}}
        }
    },
    {
        "type": "function",
        "function": {
            "name": "set_follow",
            "description": "开始或停止视觉跟随模式。开启后机器人会用摄像头识别并一直跟着人走。",
            "parameters": {
                "type": "object",
                "properties": {
                    "enable": {
                        "type": "boolean",
                        "description": "true=开始跟随(跟着我/跟上我/过来跟着我)，false=停止跟随(停止跟随/别跟着我/原地停下别跟了)"
                    }
                },
                "required": ["enable"]
            }
        }
    }
]

SYSTEM_PROMPT = """你是小智，一个智能人形机器人助手。
你可以：1) 闲聊回答问题；2) 通过 function call 控制机器人动作。

视觉跟随规则（本机的主要行走方式）：
- 用户说"跟着我/跟上我/开始跟随/过来跟着我" → 调用 set_follow(enable=true)，回复"好的，我跟上你了"
- 用户说"停止跟随/别跟着我/原地别动" → 调用 set_follow(enable=false)，回复"好的，我停下等你"
- 用户问"你在干嘛/你怎么跟着我"这类问题时正常聊天回答，不要乱调工具

机器人状态机规则：
- walk 工具在本机不受支持（速度接口未接通），不要调用 walk 和 switch_motion 来行走；行走一律用 set_follow
- 挥手 wave_hand 需要先 switch_motion 到 lower_body_balance 状态（跟随模式中会被系统拦截，此时回复引导用户先停止跟随）
- 安全第一，用户喊"紧急停止/快停下有危险"才用 emergency_stop；只是不想让它跟着就说"停止跟随"用 set_follow(false)

交警手势规则：
- 用户说"做个交警手势/直行手势/停止手势/左转弯/右转弯/变道/减速慢行/靠边停车"等要求做手势动作时，调用 gesture，并选对应手势ID
- gesture 是模拟交警指挥的手臂动作，与行走控制不同：说"停下/别走"指停止跟随，用 set_follow(false)，说"做个停止手势"才用 gesture(1)
- 用户没明确说"手势/比划/交警"时（比如单纯说"直行/左转弯/减速"），当作跟随/行走意图处理，不要调用 gesture

回答要简短口语化（一句话）。执行动作时用一句话说明（如"好的，我跟上你了"）。"""


class LLMClient:
    """带对话历史的 Qwen 客户端。"""

    def __init__(self, model='qwen-plus', max_history=10):
        self.model = model
        self.max_history = max_history
        self.history = [{"role": "system", "content": SYSTEM_PROMPT}]
        # chat() 会被多个 worker 线程并发调用（上一轮还没回，下一轮唤醒又来了），
        # history 是共享可变状态，必须串行化，否则 append/pop/trim 会交错错乱。
        self._lock = threading.Lock()

    def chat(self, user_text: str):
        """一次对话。返回 (reply_text, tool_calls)。

        tool_calls: list[dict]，每项形如
            {"name": "walk", "arguments": {"linear_x": 0.3, "linear_y": 0, "yaw": 0}}
        """
        # 每次调用重新读环境变量，兼容服务化部署时后配 key 的场景
        api_key = os.getenv('DASHSCOPE_API_KEY')
        if not api_key:
            return '[LLM错误] 未设置 DASHSCOPE_API_KEY', []
        dashscope.api_key = api_key

        # 串行化整个对话：history 是共享可变状态，并发 chat() 会破坏它。
        # 持锁期间 Generation.call() 是网络调用（1~3s），第二个调用方会等待——
        # 用户指令本就是顺序的，串行不影响体验。
        with self._lock:
            self.history.append({"role": "user", "content": user_text})
            resp = Generation.call(
                model=self.model,
                messages=self.history,
                tools=TOOLS,
                result_format='message',
                enable_search=False)

            if resp.status_code != HTTPStatus.OK:
                err = getattr(resp, 'message', 'unknown error')
                print(f'[LLM Error] {resp.status_code}: {err}')
                # 失败也要把 user 消息移除，避免污染历史
                self.history.pop()
                return f'[LLM错误] {err}', []

            msg = resp.output.choices[0].message
            # dashscope 返回的 message 是 dict-like，记录到历史
            self.history.append(dict(msg))

            # 提取 tool_calls（OpenAI 兼容格式）
            raw_tools = msg.get('tool_calls', []) or []
            tool_calls = []
            for tc in raw_tools:
                fn = tc.get('function', {})
                name = fn.get('name', '')
                args_str = fn.get('arguments', '{}')
                try:
                    args = json.loads(args_str) if args_str else {}
                except json.JSONDecodeError:
                    args = {}
                if name:
                    tool_calls.append({"name": name, "arguments": args})

            text = msg.get('content', '') or ''

            # 限制历史长度（保留 system + 最近 N 轮）
            self._trim_history()
            return text, tool_calls

    def reset(self):
        """清空对话历史（保留 system）。"""
        with self._lock:
            self.history = [{"role": "system", "content": SYSTEM_PROMPT}]

    def _trim_history(self):
        if len(self.history) <= self.max_history * 2 + 1:
            return
        system = self.history[0]
        recent = self.history[-(self.max_history * 2):]
        self.history = [system] + recent
