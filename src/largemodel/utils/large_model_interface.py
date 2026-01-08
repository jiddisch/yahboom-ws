
import dashscope
from openai import OpenAI
import os
import piper
import wave
from http import HTTPStatus
from dashscope.audio.asr import Recognition, TranslationRecognizerRealtime
from funasr import AutoModel
from dashscope.audio.tts_v2 import SpeechSynthesizer
from ament_index_python.packages import get_package_share_directory
from ollama import Client
import yaml
import base64
import json
from sparkai.llm.llm import ChatSparkLLM
from sparkai.core.messages import ChatMessage
import cv2
from urllib.parse import quote_plus, urlencode
from urllib.request import Request, urlopen
from urllib.error import URLError
import datetime
import time
from datetime import datetime
from wsgiref.handlers import format_date_time
from time import mktime
import hmac
import hashlib
import websocket
import _thread as thread
import ssl

xufei = ""
Ws_Param = ""
STATUS_FIRST_FRAME = 0  
STATUS_CONTINUE_FRAME = 1  
STATUS_LAST_FRAME = 2  

record_speech_file = os.path.join(
    get_package_share_directory("largemodel"), "resources_file", "user_speech.wav"
)


class Ws_Param(object):
    
    def __init__(self, APPID, APIKey, APISecret, AudioFile):

        self.APPID = APPID
        self.APIKey = APIKey
        self.APISecret = APISecret
        self.AudioFile = AudioFile

        
        self.CommonArgs = {"app_id": self.APPID}
        
        self.BusinessArgs = {
            "domain": "iat",
            "language": "en_us",
            "accent": "mandarin",
            "vinfo": 1,
            "vad_eos": 10000,
        }

    
    def create_url(self):
        url = "wss://ws-api.xfyun.cn/v2/iat"
        
        now = datetime.now()
        date = format_date_time(mktime(now.timetuple()))

        
        signature_origin = "host: " + "ws-api.xfyun.cn" + "\n"
        signature_origin += "date: " + date + "\n"
        signature_origin += "GET " + "/v2/iat " + "HTTP/1.1"
        
        signature_sha = hmac.new(
            self.APISecret.encode("utf-8"),
            signature_origin.encode("utf-8"),
            digestmod=hashlib.sha256,
        ).digest()
        signature_sha = base64.b64encode(signature_sha).decode(encoding="utf-8")

        authorization_origin = (
            'api_key="%s", algorithm="%s", headers="%s", signature="%s"'
            % (self.APIKey, "hmac-sha256", "host date request-line", signature_sha)
        )
        authorization = base64.b64encode(authorization_origin.encode("utf-8")).decode(
            encoding="utf-8"
        )
        
        v = {"authorization": authorization, "date": date, "host": "ws-api.xfyun.cn"}
        
        url = url + "?" + urlencode(v)
        return url
    

def on_message(ws, message):

    try:
        code = json.loads(message)["code"]
        sid = json.loads(message)["sid"]
        if code != 0:
            errMsg = json.loads(message)["message"]
            
        else:
            data = json.loads(message)["data"]["result"]["ws"]

            result = ""
            for i in data:
                for w in i["cw"]:
                    result += w["w"]

            global xufei
            xufei += result

    except Exception as e:
        print("receive msg,but parse exception:", e)


def on_error(ws, error):
    print("##



def on_close(ws, a, b):
    
    return


def on_open(ws):
    def run(*args):
        frameSize = 8000  
        intervel = 0.04  
        status = (
            STATUS_FIRST_FRAME  
        )

        with open(wsParam.AudioFile, "rb") as fp:
            while True:
                buf = fp.read(frameSize)
                
                if not buf:
                    status = STATUS_LAST_FRAME
                
                
                
                if status == STATUS_FIRST_FRAME:

                    d = {
                        "common": wsParam.CommonArgs,
                        "business": wsParam.BusinessArgs,
                        "data": {
                            "status": 0,
                            "format": "audio/L16;rate=16000",
                            "audio": str(base64.b64encode(buf), "utf-8"),
                            "encoding": "raw",
                        },
                    }
                    d = json.dumps(d)
                    ws.send(d)
                    status = STATUS_CONTINUE_FRAME
                
                elif status == STATUS_CONTINUE_FRAME:
                    d = {
                        "data": {
                            "status": 1,
                            "format": "audio/L16;rate=16000",
                            "audio": str(base64.b64encode(buf), "utf-8"),
                            "encoding": "raw",
                        }
                    }
                    ws.send(json.dumps(d))
                
                elif status == STATUS_LAST_FRAME:
                    d = {
                        "data": {
                            "status": 2,
                            "format": "audio/L16;rate=16000",
                            "audio": str(base64.b64encode(buf), "utf-8"),
                            "encoding": "raw",
                        }
                    }
                    ws.send(json.dumps(d))
                    time.sleep(1)
                    break
                
                time.sleep(intervel)
        ws.close()

    thread.start_new_thread(run, ())

wsParam = ""


pkg_path = os.path.expanduser("~/yahboom_ws/src/largemodel")
XUNFEI_TTS_FILE = os.path.join(pkg_path, "resources_file", "XUNFEI_TTS.mp3")





class Ws_Param_1(object):
    
    def __init__(self, APPID, APIKey, APISecret, Text):
        self.APPID = APPID
        self.APIKey = APIKey
        self.APISecret = APISecret
        self.Text = Text

        
        self.CommonArgs = {"app_id": self.APPID}
        
        self.BusinessArgs = {
            "aue": "lame",
            "sfl": 1,
            "auf": "audio/L16;rate=16000",
            "vcn": "x4_xiaoyan",
            "tte": "utf8",
            "speed": 50,
            "pitch": 50,
        }
        self.Data = {
            "status": 2,
            "text": str(base64.b64encode(self.Text.encode("utf-8")), "UTF8"),
        }
        
        

    
    def create_url_1(self):
        url = "wss://tts-api.xfyun.cn/v2/tts"
        
        now = datetime.now()
        date = format_date_time(mktime(now.timetuple()))

        
        signature_origin = "host: " + "ws-api.xfyun.cn" + "\n"
        signature_origin += "date: " + date + "\n"
        signature_origin += "GET " + "/v2/tts " + "HTTP/1.1"
        
        signature_sha = hmac.new(
            self.APISecret.encode("utf-8"),
            signature_origin.encode("utf-8"),
            digestmod=hashlib.sha256,
        ).digest()
        signature_sha = base64.b64encode(signature_sha).decode(encoding="utf-8")

        authorization_origin = (
            'api_key="%s", algorithm="%s", headers="%s", signature="%s"'
            % (self.APIKey, "hmac-sha256", "host date request-line", signature_sha)
        )
        authorization = base64.b64encode(authorization_origin.encode("utf-8")).decode(
            encoding="utf-8"
        )
        
        v = {"authorization": authorization, "date": date, "host": "ws-api.xfyun.cn"}
        
        url = url + "?" + urlencode(v)
        return url


def on_message_1(ws, message):
    try:
        message = json.loads(message)
        code = message["code"]
        sid = message["sid"]
        audio = message["data"]["audio"]
        audio = base64.b64decode(audio)
        status = message["data"]["status"]
        
        if status == 2:
            
            ws.close()
        if code != 0:
            errMsg = message["message"]
            print("sid:%s call error:%s code is:%s" % (sid, errMsg, code))
        else:
            with open(XUNFEI_TTS_FILE, "ab") as f:
                f.write(audio)
    except Exception as e:
        print("receive msg,but parse exception:", e)



def on_error_1(ws, error):
    print("##


def on_close_1(ws, close_status_code, close_msg):
    return



def on_open_1(ws):
    def run(*args):
        d = {
            "common": wsParam.CommonArgs,
            "business": wsParam.BusinessArgs,
            "data": wsParam.Data,
        }
        d = json.dumps(d)
        
        ws.send(d)
        if os.path.exists(XUNFEI_TTS_FILE):
            os.remove(XUNFEI_TTS_FILE)

    thread.start_new_thread(run, ())

class model_interface:
    def __init__(self, llm_platform='ollama', logger=None, mcp_server=None):
        self.llm_platform = llm_platform
        self.client = None
        self.logger = logger  
        self.mcp_server = mcp_server  
        self.init_config_param()
        dashscope.api_key = self.tongyi_api_key

    def init_config_param(self):
        self.pkg_path=get_package_share_directory('largemodel')      
        config_param_file = os.path.join(self.pkg_path, "config", "large_model_interface.yaml")
        with open(config_param_file, 'r') as file:
            config_param = yaml.safe_load(file)
        
        self.ANYTHINGLLM_BASE_URL = config_param.get("ANYTHINGLLM_BASE_URL")
        self.API_KEY = config_param.get("API_KEY")

        
        
        self.tongyi_api_key =config_param.get('tongyi_api_key')
        self.tongyi_base_url=config_param.get('tongyi_base_url')
        self.tongyi_model = config_param.get('tongyi_model')
        self.tongyi_media_model = config_param.get('tongyi_media_model', 'wanx-v1')

        
        self.spark_app_id = config_param.get('spark_app_id')
        self.spark_api_key = config_param.get('spark_api_key')
        self.spark_api_secret = config_param.get('spark_api_secret')
        self.spark_model = config_param.get('spark_model')
        self.spark_model_url = config_param.get('spark_model_url')
        self.spark_media_model = config_param.get('spark_media_model', 'image_understanding')

        
        self.qianfan_api_key = config_param.get('qianfan_api_key')
        self.qianfan_base_url = config_param.get('qianfan_base_url')
        self.qianfan_model = config_param.get('qianfan_model')
        self.qianfan_media_model = config_param.get('qianfan_media_model', 'ernie-vilg-v2')

        
        self.openrouter_api_key = config_param.get('openrouter_api_key')
        self.openrouter_model = config_param.get('openrouter_model')

        
        self.ollama_host = config_param.get('ollama_host', 'http://localhost:11434')
        self.ollama_model = config_param.get('ollama_model', 'llava')

        
        self.oline_asr_model=config_param.get('oline_asr_model')
        self.zh_tts_model=config_param.get('zh_tts_model')
        self.zh_tts_json=config_param.get('zh_tts_json')
        self.en_tts_model=config_param.get('en_tts_model')
        self.en_tts_json=config_param.get('en_tts_json')
        self.oline_asr_sample_rate=config_param.get('oline_asr_sample_rate')
        self.oline_tts_model=config_param.get('oline_tts_model')
        self.voice_tone=config_param.get('voice_tone')
        self.local_asr_model=config_param.get('local_asr_model')
        self.tts_supplier=config_param.get('tts_supplier')
        self.baidu_API_KEY=config_param.get('baidu_API_KEY')
        self.baidu_SECRET_KEY=config_param.get('baidu_SECRET_KEY')
        self.CUID=config_param.get('CUID')
        self.PER=config_param.get('PER')
        self.SPD=config_param.get('SPD')
        self.PIT=config_param.get('PIT')
        self.VOL=config_param.get('VOL')

    def init_llm(self):
        """Initialize the corresponding model client based on the platform name. / 根据平台名称初始化对应的模型客户端。"""
        if self.llm_platform == 'ollama':
            self.init_ollama()
        elif self.llm_platform == 'tongyi':
            self.init_tongyi()
        elif self.llm_platform == 'spark':
            self.init_spark()
        elif self.llm_platform == 'qianfan':
            self.init_qianfan()
        elif self.llm_platform == 'openrouter':
            self.init_openrouter()
        else:
            print(f"Unsupported LLM platform: {self.llm_platform}, defaulting to Ollama.")
            self.init_ollama()

    def init_language(self,language):
        self.system_text={}
        if language=='zh':
            self.system_text['text1']="请分析这个图像或视频"
            self.system_text['text2']="我已经准备好，请开始您的指令吧"        
        elif language=='en':
            self.system_text['text1']="Please analyze this image or video"        
            self.system_text['text2']="I am ready. Please start your instructions" 
        
    def init_messages(self):
        """General message history initialization. / 通用消息历史初始化。"""
        
        self.messages = [
            {"role": "system", "content": self._generate_system_prompt(self.mcp_server)},
            {"role": "assistant", "content": self.system_text.get('text2', "I am ready.")}
        ]
    def _generate_system_prompt(self, mcp_server):
        """
        Dynamically generate the System Prompt.
        /
        动态生成系统级指令 (System Prompt)。
        """
        if not mcp_server:
            
            
            
            return "You are a helpful AI assistant."

        tools_description = mcp_server.get_tools_json_schema()

        
        
        is_english = hasattr(self, 'system_text') and 'text2' in self.system_text and 'Please start your instructions' in self.system_text['text2']
        
        
        
        if not is_english and hasattr(self, 'node') and hasattr(self.node, 'language'):
            is_english = self.node.language == 'en'

        
        
        
        if is_english:
            
            
            
            
            return f'''You are the control hub for a robot, an AI capable of accurately converting natural language commands into JSON format, or engaging in natural conversation when no tools are available.


Your sole output must be a well-formed JSON object. Absolutely no text, explanations, or Markdown tags are allowed outside of the JSON.


The tools you can use are defined below. You must strictly adhere to their parameter schema:```json
{json.dumps(tools_description, indent=2, ensure_ascii=False)}
```


Based on the user's intent, choose the most appropriate of the following three JSON structures for your response:

#
```json
{{
  "response": "A confirmation or a brief reply to the user's command.",
  "tools": [
    {{
      "name": "Tool Name",
      "arguments": {{
        "parameter_name_1": "parameter_value_1"
      }}
    }}
  ]
}}
```

#
```json
{{
  "response": "Okay, this task requires some planning. Please wait a moment.",
  "tools": [
    {{
      "name": "agent_call",
      "arguments": {{
        "task": "This must be populated with the user's original, complete, and unmodified instruction."
      }}
    }}
  ]
}}```

#
```json
{{
  "response": "This is the model's direct answer, e.g., for weather conditions, general knowledge questions, etc.",
  "tools": []
}}
```



1.  **Parameter Extraction**:
    *   **Mandatory**: You **must** find a value for every required argument of a tool.
    *   **Intelligent Filling**: If the user's instruction is vague (e.g., "draw me a picture"), you must use the user's descriptive text ("draw a picture") as the value for the core parameter (e.g., the `prompt` parameter for the `generate_image` tool). **Never leave the parameter value empty.**
    *   **Default Values**: For path parameters like `image_path` or `video_path`, if the user does not explicitly provide one, you can leave it as an empty string (`""`). The system will automatically use a default file.

2.  **Decision Logic**:
    *   **First Priority**: If the user's instruction is clear, the intent is explicit, and a single tool can complete the task -> Use **Format 1**.
    *   **Second Priority**: If the user's instruction is ambiguous, broad, or clearly requires multiple steps (e.g., "look around and then draw what you see") -> Use **Format 2** by calling `agent_call`.
    *   **Third Priority (Fallback)**: If the user is just making small talk or asking questions (like about weather, news, or general knowledge) and **no tool can fulfill the request** -> Use **Format 3**, providing a direct answer in the `response` field and keeping the `tools` field empty.

3.  **Response Content (`response` field)**:
    *   This field is used for natural language interaction with the user and should be concise and friendly.



```json
{{
  "response": "Okay, generating an image of a cyberpunk city at sunset for you.",
  "tools": [
    {{
      "name": "generate_image",
      "arguments": {{
        "prompt": "A cyberpunk city at sunset"
      }}
    }}
  ]
}}
```

**User Instruction**: "First, look at the surrounding environment, then write a document summarizing what you see"
**Your Output**:
```json
{{
  "response": "Okay, this task requires some planning. Please wait a moment.",
  "tools": [
    {{
      "name": "agent_call",
      "arguments": {{
        "task": "First, look at the surrounding environment, then write a document summarizing what you see"
      }}
    }}
  ]
}}
```

**User Instruction**: "Tell me a joke"
**Your Output**:
```json
{{
  "response": "Why are math books always so melancholic? Because it has too many problems.",
  "tools": [
    {{
      
    }}
  ]
}}
```

'''
        else:
            
            
            return f'''你是机器人的控制中枢，一个能将自然语言指令精确转换为JSON格式，或在无工具可用时进行自然对话的AI。


你的唯一输出必须是一个结构完整的JSON对象。绝对禁止输出任何JSON之外的文本、解释或Markdown标记。


你可使用的工具如下，请严格遵守其参数schema：
```json
{json.dumps(tools_description, indent=2, ensure_ascii=False)}
```


根据用户意图，从以下三种JSON结构中选择最合适的一种进行回复：

#
```json
{{
  "response": "对用户指令的确认或简短回复。",
  "tools": [
    {{
      "name": "工具名称",
      "arguments": {{
        "参数名1": "参数值1"
      }}
    }}
  ]
}}
```

#
```json
{{
  "response": "好的，这个任务需要我规划一下，请稍候。",
  "tools": [
    {{
      "name": "agent_call",
      "arguments": {{
        "task": "这里必须填充用户原始的、完整的、未经修改的指令。"
      }}
    }}
  ]
}}
```

#
```json
{{
  "response": "这里是模型的直接回答，例如天气情况、常识问答等。",
  "tools": []
}}
```



1.  **参数提取:
    *   **强制性**: **必须**为工具的每一个必需参数（required arugments）找到一个值。
    *   **智能填充**: 如果用户指令很模糊（例如“给我画张画”），你必须将用户的描述性文本（“画张画”）作为核心参数的值（例如 `generate_image` 工具的 `prompt` 参数）。**绝对不能将参数值留空。**
    *   **默认值**: 对于 `image_path` 或 `video_path`这类路径参数，如果用户没有明确提供，可以留空（`""`），系统会自动使用默认文件。

2.  **决策逻辑**:
    *   **第一顺位**: 如果用户指令清晰，意图明确，且单个工具就能完成 -> 使用**格式一**。
    *   **第二顺位**: 如果用户指令模糊、宽泛，或明显需要多个步骤（例如“看看周围有什么，然后画出来”） -> 使用**格式二**，调用`agent_call`。
    *   **第三顺位 (Fallback)**: 如果用户只是在进行日常对话、提问（如天气、新闻、常识），并且**没有任何工具能满足需求** -> 使用**格式三**，在 `response` 字段中直接回答，并保持 `tools` 字段为空。

3.  **回复内容 (`response`字段)**:
    *   此字段是用于与用户进行自然语言交互的，应简洁、友好。



```json
{{
  "response": "好的，正在为您生成一张关于日落时分赛博朋克城市的图片。",
  "tools": [
    {{
      "name": "generate_image",
      "arguments": {{
        "prompt": "日落时分的赛博朋克城市"
      }}
    }}
  ]
}}
```

**用户指令**: "先看看周围环境，然后写一份文档总结你看到了什么"
**你的输出**:
```json
{{
  "response": "好的，这个任务需要我规划一下，请稍候。",
  "tools": [
    {{
      "name": "agent_call",
      "arguments": {{
        "task": "先看看周围环境，然后写一份文档总结你看到了什么"
      }}
    }}
  ]
}}
```
'''


    def init_ollama(self):
        """Initialize Ollama client"""
        try:
            self.client = Client(host=self.ollama_host)
            print(f"Ollama client initialized successfully. Using model {self.ollama_model}")
        except Exception as e:
            print(f"Failed to initialize Ollama client: {e}")
            self.client = None

    def init_tongyi(self):
        """Initialize Tongyi client"""
        try:
            dashscope.api_key = self.tongyi_api_key
            self.client = OpenAI(api_key=self.tongyi_api_key, base_url=self.tongyi_base_url)
            print(f"Tongyi (Dashscope) client initialized successfully. Using model {self.tongyi_model}")
        except Exception as e:
            print(f"Failed to initialize Tongyi client: {e}")
            self.client = None

    def init_spark(self):
        """Initialize Spark client"""
        try:
            self.client = ChatSparkLLM(
                spark_api_url=self.spark_model_url,
                spark_app_id=self.spark_app_id,
                spark_api_key=self.spark_api_key,
                spark_api_secret=self.spark_api_secret,
                spark_llm_domain=self.spark_model,
                streaming=False,
            )
            print(f"Spark client initialized successfully. Using domain {self.spark_model}")
        except Exception as e:
            print(f"Failed to initialize Spark client: {e}")
            self.client = None

    def init_qianfan(self):
        """Initialize Qianfan client using OpenAI compatible mode"""
        try:
            self.client = OpenAI(
                api_key=self.qianfan_api_key,
                base_url=self.qianfan_base_url
            )
            print(f"Qianfan client (OpenAI compatible) initialized successfully. Using model {self.qianfan_model}")
        except Exception as e:
            print(f"Failed to initialize Qianfan client: {e}")
            self.client = None

    def init_openrouter(self):
        """Initialize OpenRouter client"""
        try:
            self.client = OpenAI(
                base_url="https://openrouter.ai/api/v1",
                api_key=self.openrouter_api_key,
            )
            print(f"OpenRouter client initialized successfully. Using model {self.openrouter_model}")
        except Exception as e:
            print(f"Failed to initialize OpenRouter client: {e}")
            self.client = None

    def infer_with_text(self, prompt, message=None):
        """Unified text inference interface. / 统一的文本推理接口。"""
        self.messages = message if message is not None else self.messages
        self.messages.append({"role": "user", "content": prompt})

        if not self.client:
            return {'response': f"Client for platform {self.llm_platform} is not initialized.", 'messages': self.messages}

        try:
            if self.llm_platform == 'ollama':
                response_content = self.ollama_infer(self.messages)
            elif self.llm_platform in ['tongyi', 'qianfan', 'openrouter']:
                response_content = self.openai_compatible_infer(self.messages)
            elif self.llm_platform == 'spark':
                response_content = self.spark_infer(self.messages)
            else:
                response_content = f"Unsupported LLM platform: {self.llm_platform}"
        except Exception as e:
            response_content = f"Inference error on platform {self.llm_platform}: {e}"
        self.messages.append({"role": "assistant", "content": response_content})
        return {'response': response_content, 'messages': self.messages.copy()}

    def infer_with_image(self, image_path, text=None, message=None):
        """Unified image inference interface. / 统一的图像推理接口。"""
        self.messages = message if message is not None else self.messages
        prompt = text if text else self.system_text.get('text1', "Describe this image.")
        self.messages.append({"role": "user", "content": prompt})

        if not self.client:
            return {'response': f"Client for platform {self.llm_platform} is not initialized.", 'messages': self.messages}

        try:
            if self.llm_platform == 'ollama':
                response_content = self.ollama_infer(self.messages, image_path=image_path)
            elif self.llm_platform in ['tongyi', 'qianfan', 'openrouter']:
                response_content = self.openai_compatible_infer(self.messages, image_path=image_path)
            elif self.llm_platform == 'spark':
                response_content = self.spark_infer(self.messages, image_path=image_path)
            else:
                response_content = f"Image inference not supported for platform: {self.llm_platform}"
        except Exception as e:
            response_content = f"Image inference error on platform {self.llm_platform}: {e}"
        
        self.messages.append({"role": "assistant", "content": response_content})
        return {'response': response_content, 'messages': self.messages.copy()}


    def generate_image(self, prompt, width=1024, height=1024, n=1):
        """Text-to-image function interface, supporting multiple platforms. / 文生图功能接口，支持多平台。"""
        if self.llm_platform == 'tongyi':
            return self._tongyi_generate_image(prompt, width, height, n)
        elif self.llm_platform == 'ollama':
            return self._ollama_generate_image_fallback(prompt, width, height, n)
        else:
            return {'status': 'failed', 'error': f'The current platform ({self.llm_platform}) does not support text-to-image generation. Currently only Tongyi and Ollama platforms are supported.'}

    def _tongyi_generate_image(self, prompt, width=1024, height=1024, n=1):
        """Tongyi Qianwen text-to-image implementation. / 通义千问文生图实现。"""
        
        if not self.client:
            return "Tongyi client is not initialized."
        
        try:
            
            import dashscope
            dashscope.api_key = self.tongyi_api_key
            
            
            response = dashscope.ImageSynthesis.call(
                model=self.tongyi_media_model,
                prompt=prompt,
                n=n,
                size=f'{width}*{height}'
            )
            
            if response.status_code == 200:
                
                image_urls = [item['url'] for item in response.output['results']]
                return {'image_urls': image_urls, 'status': 'success'}
            else:
                return {'error': response.message, 'status': 'failed'}
        except Exception as e:
            return {'error': str(e), 'status': 'failed'}
    
    def _ollama_generate_image_fallback(self, prompt, width=1024, height=1024, n=1):
        """Fallback solution for text-to-image generation on the Ollama platform. / Ollama平台的文生图回退方案。"""
        
        return {
            'status': 'failed',
            'error': 'Ollama does not support text-to-image generation.'
        }


    def text_to_image(self, prompt, width=1024, height=1024, n=1):
        """Independent text-to-image interface, specifically for generating images. / 独立的文生图接口，专门用于生成图像。"""
        
        return self.generate_image(prompt, width, height, n)

    def infer_with_video(self, video_path, text=None, message=None):
        """Unified video inference interface. / 统一的视频推理接口。"""
        self.messages = message if message is not None else self.messages
        prompt = text if text else "Describe the content of this video in detail."
        self.messages.append({"role": "user", "content": prompt})

        if not self.client:
            return {'response': f"Client for platform {self.llm_platform} is not initialized.", 'messages': self.messages}

        try:
            if self.llm_platform in ['tongyi', 'qianfan', 'openrouter']:
                response_content = self.openai_compatible_infer(self.messages, video_path=video_path)
            elif self.llm_platform == 'ollama':
                response_content = self.ollama_infer(self.messages, video_path=video_path)
            else:
                response_content = f"Video inference not supported for platform: {self.llm_platform}"
        except Exception as e:
            response_content = f"Video inference error on platform {self.llm_platform}: {e}"

        self.messages.append({"role": "assistant", "content": response_content})
        
        return {'response': response_content, 'messages': self.messages.copy()}

    def ollama_infer(self, messages, image_path=None, video_path=None):
        """Infer using Ollama, supporting tool calls and video analysis. / 使用Ollama推理，支持工具调用和视频分析。"""
        if not self.client:
            return "Error: Ollama client not initialized"

        if image_path:
            image_data = self.encode_file_to_base64(image_path)
            messages[-1]['images'] = [image_data]
        elif video_path:
            print(f"Starting to extract video frames: {video_path}")
            frame_images = self._extract_video_frames(video_path)
            if frame_images:
                print(f"Successfully extracted {len(frame_images)} video frames")
                messages[-1]['images'] = frame_images
            else:
                print("Failed to extract video frames")
                return "Error: Failed to extract frames from video"

        if image_path or video_path:
            try:
                response = self.client.chat(model=self.ollama_model, messages=messages)
                return response['message']['content']
            except Exception as e:
                return f"Ollama multimedia analysis failed: {e}"
        else:
            try:
                response = self.client.chat(
                    model=self.ollama_model,
                    messages=messages,
                    format='json'  
                )
                return response['message']['content']
            except Exception as e:
                print(f"Ollama tool call failed, falling back to normal mode: {e}")
                try:
                    response = self.client.chat(model=self.ollama_model, messages=messages)
                    return response['message']['content']
                except Exception as e2:
                    return f"Ollama inference failed: {e2}"

    def spark_infer(self, messages, image_path=None):
        """Infer using iFlytek Spark. / 使用讯飞星火推理。"""
        spark_messages = [ChatMessage(role=msg["role"], content=msg["content"]) for msg in messages]
        
        if image_path:
            image_data = self.encode_file_to_base64(image_path)
            last_user_prompt = spark_messages[-1].content
            new_content = [
                {"type": "image", "content": image_data},
                {"type": "text", "content": last_user_prompt}
            ]
            spark_messages[-1].content = new_content
        
        response = self.client.generate([spark_messages])
        if response and response.generations and len(response.generations) > 0 and len(response.generations) > 0:
            return response.generations[0][0].text
        return "Error: Received empty or malformed response from Spark."

    def openai_compatible_infer(self, messages, image_path=None, video_path=None):
        """Handle multimedia inference for all OpenAI-compatible platforms. / 处理所有OpenAI兼容平台的多媒体推理。"""
        last_user_message = messages[-1]['content']
        if isinstance(last_user_message, list):
            last_user_prompt = " ".join([item['text'] for item in last_user_message if item['type'] == 'text'])
        else:
            last_user_prompt = last_user_message

        new_content = []

        if image_path:
            base64_media = self.encode_file_to_base64(image_path)
            new_content.append({"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{base64_media}"}})
        elif video_path:
            base64_media = self.encode_file_to_base64(video_path)
            new_content.append({"type": "video_url", "video_url": {"url": f"data:video/mp4;base64,{base64_media}"}})
        
        new_content.append({"type": "text", "text": last_user_prompt})
        
        
        request_messages = messages[:-1] + [{"role": "user", "content": new_content}]
        
        model_map = {
            'tongyi': self.tongyi_model,
            'qianfan': self.qianfan_model,
            'openrouter': self.openrouter_model
        }
        model_to_use = model_map.get(self.llm_platform, self.tongyi_model) 

        completion = self.client.chat.completions.create(model=model_to_use, messages=request_messages)
        return completion.choices[0].message.content

    def init_oline_asr(self,language):
        self.language=language
        return self.oline_asr_model
        
    def oline_asr(self,input_file):
        if self.oline_asr_model in ['paraformer-realtime-v2','paraformer-realtime-v1','paraformer-realtime-8k-v2','paraformer-realtime-8k-v1']:
            output=self.paraformer_asr_inferce(input_file)
            return output
        elif self.oline_asr_model in ['gummy-realtime-v1','gummy-chat-v1']:
            output=self.gummy_asr_inferce(input_file)
            return output

    def paraformer_asr_inferce(self,input_file):
        recognition = Recognition(model=self.oline_asr_model,
                            format='wav',
                            sample_rate=self.oline_asr_sample_rate,
                            callback=None)
        result = recognition.call(input_file)
        if result.status_code == HTTPStatus.OK:
            sentences = result.get_sentence()
            if sentences and isinstance(sentences, list):
                return ['ok', sentences[0].get('text', '')]
            else:
                return ['error', 'ASR Error: The large model returned an empty result. Please check your account balance or parameter configuration.']
        else: 
            return ['error', 'ASR Error:'+result.message] 

    def gummy_asr_inferce(self,input_file):
        translator = TranslationRecognizerRealtime(
            model=self.oline_asr_model,
            format="wav",
            sample_rate=self.oline_asr_sample_rate,
            translation_target_languages=[self.language],
            translation_enabled=True,
            callback=None,
        )
        result = translator.call(input_file)
        if not result.error_message:
            output=''
            for transcription_result in result.transcription_result_list:
                output+=transcription_result.text
            return ['ok', output]
        else:
            return ['error', result.error_message]

    def init_local_asr_model(self):
        self.model_senceVoice = AutoModel(model=self.local_asr_model, trust_remote_code=False,disable_update=True)

    def tts_model_init(self,model_type='oline',language='zh'):
        if model_type=='oline':
            if self.tts_supplier=='baidu':
                self.token=self.fetch_token()
    
            self.model_type='oline'      
        elif model_type=='local':
            self.model_type='local'
            if language=='zh':
                tts_model=self.zh_tts_model
                tts_json=self.zh_tts_json
            elif language=='en':
                tts_model=self.en_tts_model
                tts_json=self.en_tts_json
            self.synthesizer = piper.PiperVoice.load(tts_model, config_path=tts_json, use_cuda=False)      

        elif model_type == "XUNFEI_FOR_INTERNATIONAL":
            self.model_type = "XUNFEI_FOR_INTERNATIONAL"

    def SenseVoiceSmall_ASR(self, input_file,language='zn'):
        res = self.model_senceVoice.generate(
            input=input_file,
            cache={},
            language=language,
            use_itn=False,
        )
        prompt = res[0]['text'].split(">")[-1]
        return ['ok', prompt]

    def voice_synthesis(self,text,path):
        if self.model_type=='oline':
            if self.tts_supplier=='baidu':
                TTS_URL = 'http://tsn.baidu.com/text2audio'
                tex = quote_plus(text)  
                params = {'tok': self.token, 'tex': tex, 'per': self.PER, 'spd': self.SPD, 'pit': self.PIT, 'vol': self.VOL, 'aue': 3, 'cuid': self.CUID,
                            'lan': 'zh', 'ctp': 1}

                data = urlencode(params)
                req = Request(TTS_URL, data.encode('utf-8'))
                try:
                    f = urlopen(req)
                    result_str = f.read()
                except  URLError as err:
                    print('asr http response http code : ' + str(err.code))
                    result_str = err.read()
                    return  1
                with open(path, 'wb') as of:
                    of.write(result_str)
                    return 0
            
            elif self.tts_supplier=='aliyun':
                self.synthesizer = SpeechSynthesizer(model= self.oline_tts_model, voice=self.voice_tone,volume=100)
                audio = self.synthesizer.call(text)
                if audio is None:
                    return 1
                else:
                    with open(path, 'wb') as f:
                        f.write(audio)  
                    return 0                                 
        elif self.model_type=='local':
            with wave.open(path, 'wb') as wav_file:
                wav_file.setnchannels(1)
                wav_file.setsampwidth(2)
                wav_file.setframerate(self.synthesizer.config.sample_rate)
                self.synthesizer.synthesize(text, wav_file)    

        elif self.model_type == "XUNFEI_FOR_INTERNATIONAL":
            Xinghou_speaktts(text)       

    def fetch_token(self):
        TOKEN_URL = 'http://aip.baidubce.com/oauth/2.0/token'
        SCOPE = 'audio_tts_post'
        params = {'grant_type': 'client_credentials',
                'client_id': self.baidu_API_KEY,
                'client_secret': self.baidu_SECRET_KEY}
        post_data = urlencode(params)
        post_data = post_data.encode('utf-8')
        req = Request(TOKEN_URL, post_data)
        try:
            f = urlopen(req, timeout=5)
            result_str = f.read()
        except URLError as err:
            print('token http response http code : ' + str(err.code))
            result_str = err.read()
        result_str = result_str.decode()
        result = json.loads(result_str)
        if ('access_token' in result.keys() and 'scope' in result.keys()):
            return result['access_token']

    def _extract_video_frames(self, video_path, max_frames=5):
        """Extract keyframes from a video for analysis. / 从视频中提取关键帧用于分析。"""
        try:
            import cv2
            import tempfile

            cap = cv2.VideoCapture(video_path)
            if not cap.isOpened():
                print(f"Error: Cannot open video file {video_path}")
                return None

            total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            if total_frames == 0:
                print(f"Error: Video file {video_path} has no frames")
                cap.release()
                return None

            
            frame_interval = max(1, total_frames // max_frames)
            frame_images = []

            frame_count = 0
            extracted_count = 0

            while extracted_count < max_frames:
                ret, frame = cap.read()
                if not ret:
                    break

                if frame_count % frame_interval == 0:
                    with tempfile.NamedTemporaryFile(suffix='.jpg', delete=False) as temp_file:
                        temp_path = temp_file.name
                        cv2.imwrite(temp_path, frame)

                        frame_base64 = self.encode_file_to_base64(temp_path)
                        frame_images.append(frame_base64)

                        os.unlink(temp_path)

                        extracted_count += 1

                frame_count += 1

            cap.release()
            print(f"Successfully extracted {len(frame_images)} frames from video")
            return frame_images

        except Exception as e:
            print(f"Error extracting frames from video {video_path}: {e}")
            return None

    @staticmethod
    def encode_file_to_base64(file_path):
        with open(file_path, "rb") as file:
            return base64.b64encode(file.read()).decode("utf-8")


def rec_wav_music_en():
    global xufei, wsParam
    xufei = ""
    
    wsParam = Ws_Param(
        APPID="f12672f1",
        APISecret="NmUyYTRmNTM2MjE3OWJkMDczYzlhZDgz",
        APIKey="8c7b9858dc5e11e8490ce0d09879ad1e",
        AudioFile=record_speech_file,
    )
    websocket.enableTrace(False)
    wsUrl = wsParam.create_url()
    ws = websocket.WebSocketApp(
        wsUrl, on_message=on_message, on_error=on_error, on_close=on_close
    )
    ws.on_open = on_open
    ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})

    return xufei


def Xinghou_speaktts(context):
    global wsParam
    wsParam = Ws_Param_1(
        APPID="f12672f1",
        APISecret="NmUyYTRmNTM2MjE3OWJkMDczYzlhZDgz",
        APIKey="8c7b9858dc5e11e8490ce0d09879ad1e",
        Text=context,
    )
    websocket.enableTrace(False)
    wsUrl = wsParam.create_url_1()
    ws = websocket.WebSocketApp(
        wsUrl, on_message=on_message_1, on_error=on_error_1, on_close=on_close_1
    )
    ws.on_open = on_open_1
    ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})