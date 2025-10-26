"""Configuration settings for Robot Dog Voice Agent"""

import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configurable timeout settings
USER_RESPONSE_TIMEOUT = int(os.getenv("USER_RESPONSE_TIMEOUT", "15"))  # seconds
FINAL_TIMEOUT = int(os.getenv("FINAL_TIMEOUT", "10"))  # seconds after prompt
MAX_CONVERSATION_TIME = int(os.getenv("MAX_CONVERSATION_TIME", "420"))  # 7 minutes total
CONVERSATION_WARNING_TIME = int(os.getenv("CONVERSATION_WARNING_TIME", "300"))  # 5 minutes - first warning
FINAL_WARNING_TIME = int(os.getenv("FINAL_WARNING_TIME", "360"))  # 6 minutes - final warning

# WebSocket server settings
WEBSOCKET_HOST = os.getenv("WEBSOCKET_HOST", "localhost")
WEBSOCKET_PORT = int(os.getenv("WEBSOCKET_PORT", "8080"))

# Required environment variables for validation
REQUIRED_ENV_VARS = ["OPENAI_API_KEY"]

# Valid emotions for the agent
VALID_EMOTIONS = {
    "excited", "helpful", "friendly", "curious", "empathetic", 
    "sleepy", "waiting", "confused", "proud", "playful", 
    "focused", "surprised", "enthusiastic", "warm", "professional", "cheerful", "excuse"
} 