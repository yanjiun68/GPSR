from llm_interfaces.srv import Llm
import json
import os
from openai import OpenAI
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import rclpy
from rclpy.node import Node

class LlmService(Node):

    def __init__(self):
        super().__init__('llm_service')
        self.srv = self.create_service(Llm, 'llm', self.llm_callback)
        
        api_key = os.environ.get("GEMINI_API_KEY")
        self.client = OpenAI(api_key=api_key, base_url="https://generativelanguage.googleapis.com/v1beta/openai/")

        share_dir = Path(get_package_share_directory("llm_pkg"))
        prompt_path = share_dir / "prompt.txt"
        self.prompt = prompt_path.read_text(encoding="utf-8")
        self.get_logger().info(f"LLM service ready. Prompt loaded from {prompt_path}")


    def llm_callback(self, request, response):
        self.get_logger().info(f"Incoming request: {request.user_input}")
        try:
            completion = self.client.chat.completions.create(
                model="gemini-3.1-flash-lite-preview",
                messages=[
                    {"role": "system", "content": self.prompt},
                    {"role": "user", "content": request.user_input},
                ],
                stream=False,
                response_format={'type': 'json_object'},
                temperature=0.5
            )
            content = completion.choices[0].message.content or "{}"
            try:
                response_json = json.loads(content)
            except json.JSONDecodeError:
                response_json = {"steps": [content]}

            steps = list(response_json.get("steps"))
            pairs = []
            for step in steps:
                step = dict(step)
                action = str(step.get("action"))
                args = dict(step.get("args"))
                values = args.values()
                if len(values) == 1:
                    pairs.append(f"{action}|{list(values)[0]}")
                else:
                    tmp = action + "|"
                    for value in values:
                        tmp += f"{value},"
                    tmp = tmp.removesuffix(",")
                    pairs.append(tmp)
            response.pairs = pairs

            self.get_logger().info(str(response.pairs))
        except Exception as exc: 
            self.get_logger().error(f"LLM call failed: {exc}")
            response.pairs = ['error, (llm_call_failed)']

        return response

def main():
    rclpy.init()

    node = LlmService()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
