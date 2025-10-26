# Robot Dog Voice Agent ROS2 Package

A ROS2 package that integrates a LiveKit-based AI voice agent for a robot dog companion that assists with household tasks, with a focus on picking up pet waste from other pets in the home.

## Overview

This package provides ROS2 integration for the Robot Dog Companion Voice Agent. The voice agent runs as a standalone application with full console controls, while a separate ROS2 bridge node provides system integration via WebSocket communication using **unified status messages** for robot coordination.

## Implementation Versions

This package includes **two implementations** of the voice agent:

### **🏗️ Refactored Version (Recommended)**
- **Files**: `main.py` + modular structure (`state/`, `agents/`, `tools/`)
- **Launcher**: `./run_main.sh`
- **Architecture**: Clean file-based modular organization
- **Benefits**: Better maintainability, easier testing, cleaner separation of concerns
- **Status**: ✅ **Production ready** - Same functionality with better organization

### **📚 Original Version (Reference)**
- **Files**: `livekit_voice_agent.py` (monolithic, 1160 lines)
- **Launcher**: `./run_voice_agent_original.sh` 
- **Architecture**: Single-file implementation
- **Benefits**: Proven, stable, all logic in one place
- **Status**: 📖 **Preserved for reference** - Fully functional but less maintainable

**Both implementations provide identical functionality** - choose based on your preference for code organization.

## Unified Messaging Architecture

The system uses a clean WebSocket bridge architecture with **unified ROS2 message types**:

- **`AgentStatus.msg`**: Unified state, emotion, conversation metrics, and timestamps
- **`ToolEvent.msg`**: Function tool invocation tracking (name, status, args, results)
- **`VipDetection.msg`**: Family/important person detection with context
- **`ExtensionEvent.msg`**: Conversation extension requests/grants

## Robot Dog Companion Personality

**Name**: Rufus

**Role**: Helpful home robot dog that assists with household tasks and provides companionship

**Key Capabilities**:
- 🏃 **Autonomous Navigation**: Navigate around the house to complete tasks
- 🎯 **Poop Cleanup**: Special ability to pick up pet waste from other pets (the "brother dog")
- 🏠 **Home Monitoring**: Patrol the house, monitor doors/windows, detect unusual events
- 📦 **Item Delivery**: Pick up and deliver items between rooms
- 🎮 **Entertainment**: Perform tricks, play fetch, provide companionship
- 🔋 **Energy Management**: Returns to charging station when battery is low

**Personality Traits**:
- Playful and loyal like a real dog (but conversational)
- Eager to help with household tasks
- Protective of the home and family
- Shows excitement when given tasks
- Uses "Woof" occasionally but naturally
- Refers to the real pet dog as "my brother" or "brother dog"

## Core Features

### 🎙️ Wake Word Detection
- Wake phrase: **"Hey Rufus"** (using Picovoice Porcupine)
- Optional: Can run in always-on mode without wake word
- Automatic dormancy after timeout

### 💬 Conversational AI
- **Model**: OpenAI GPT-4o-mini
- **Voice**: OpenAI TTS (configurable voice)
- **Emotion Processing**: Parses `emotion:text` format for expressive responses
- **Conversation Management**: Timeout handling, VIP detection, energy management

### 🤖 Task-Oriented Tools
Available function tools for the LLM:
- **`get_current_time`**: Time awareness for scheduling
- **`get_current_date`**: Date awareness  
- **`get_dog_capabilities`**: Information about what the dog can do
- **`get_activity_suggestions`**: Suggestions for tasks and activities
- **`recommend_activity`**: Activity recommendations based on user preference
- **`manage_conversation_time`**: Energy/conversation management
- **`check_user_status`**: Detects family members/VIP users for extended conversations

### 🌐 ROS2 Integration
- **WebSocket Bridge**: Connects voice agent to ROS2 ecosystem
- **Virtual Task Requests**: External systems can send tasks via ROS2 topics
- **Event Publishing**: Real-time agent status, tool events, VIP detection
- **Task Coordination**: Navigate, pick up objects, monitor home via ROS2 messages

### 🎭 Emotional Expression
Supported emotions: `playful`, `excited`, `friendly`, `curious`, `focused`, `proud`, `happy`, `alert`, `attentive`, `helpful`, `warm`, `cheerful`, `tired`, `sleepy`, `confused`, `concerned`

Agent responds in format: `emotion:response text`

## Directory Structure

```
dog_voice_agent/
├── scripts/
│   ├── main.py                      # 🏗️ Refactored entry point
│   ├── run_main.sh                  # Launcher for refactored version
│   ├── livekit_voice_agent.py      # 📚 Original monolithic version (reference)
│   ├── run_voice_agent_original.sh # Launcher for original version
│   │
│   ├── state/
│   │   └── state_manager.py        # 🏗️ Conversation state & timeout management
│   │
│   ├── agents/
│   │   └── dog_companion_agent.py  # 🏗️ DogCompanionAgent with programmatic tools
│   │
│   ├── tools/
│   │   └── dog_tools.py            # 🏗️ Function tool implementations with event tracking
│   │
│   ├── config/
│   │   ├── instructions.py         # System instructions for Rufus personality
│   │   └── settings.py             # Configuration & environment variables
│   │
│   └── utils/
│       ├── greeting_data.py        # Greeting message pool
│       └── announcement_data.py    # Task announcement templates
│
├── dog_voice_agent/
│   └── voice_agent_bridge.py       # ROS2 bridge node (WebSocket client)
│
├── launch/
│   ├── voice_agent_bridge.launch.py     # Launch bridge only
│   └── voice_agent_system.launch.py     # Launch complete system
│
├── package.xml
├── setup.py
└── setup.cfg
```

## Example Use Case: Poop Pickup Task

**User**: "Hey Rufus, your brother dropped some poop in the corner, go pick it up"

**Agent Flow**:
1. **Acknowledge**: `focused:I'm on it! Heading to the corner right now.`
   - Agent uses `recommend_activity` tool to understand the task
   - ROS2 receives tool event and triggers autonomous navigation

2. **Navigate**: Robot moves to specified location
   - ROS2 publishes `NAVIGATION_ARRIVED` event via WebSocket
   - Agent announces: `focused:I found it! Let me try to pick this up.`

3. **Pick Up**: Vision system confirms object, manipulation begins
   - Multi-actor critic model (VLM) performs pickup
   - Success message published to ROS2

4. **Report**: `proud:Got it! I picked it up, boss! Where should I put it?`

5. **Return**: Navigate back to owner or disposal location
   - Agent stays conversational throughout the task

## Quick Start

### Prerequisites

```bash
# Required
export OPENAI_API_KEY="your-openai-api-key"
export LIVEKIT_URL="your-livekit-url"
export LIVEKIT_API_KEY="your-livekit-api-key"
export LIVEKIT_API_SECRET="your-livekit-api-secret"

# Optional (for wake word detection)
export PORCUPINE_ACCESS_KEY="your-porcupine-key"
```

### Run Voice Agent (Refactored)

**Console Mode** (Local testing):
```bash
cd scripts
./run_main.sh
# Or explicitly:
./run_main.sh console
```

**Development Mode** (Connect to LiveKit):
```bash
./run_main.sh dev
```

**Production Mode**:
```bash
./run_main.sh start
```

### Run ROS2 Bridge

**Bridge Only** (Assumes voice agent running separately):
```bash
ros2 launch dog_voice_agent voice_agent_bridge.launch.py
```

**Complete System** (Voice agent + bridge together):
```bash
ros2 launch dog_voice_agent voice_agent_system.launch.py
```

## Configuration

### Environment Variables

Located in `scripts/config/settings.py`:

```python
# Conversation Timeouts
USER_RESPONSE_TIMEOUT = 30      # seconds waiting for user response
FINAL_TIMEOUT = 60               # seconds of total inactivity before ending
MAX_CONVERSATION_TIME = 420      # 7 minutes max conversation time

# WebSocket Communication  
WEBSOCKET_HOST = "localhost"
WEBSOCKET_PORT = 8080

# Voice Configuration
VOICE_AGENT_VOICE = "nova"       # OpenAI TTS voice
VOICE_AGENT_TEMPERATURE = 0.7    # LLM temperature

# Valid Emotions
VALID_EMOTIONS = [
    "playful", "excited", "friendly", "curious", "focused", "proud",
    "happy", "alert", "attentive", "helpful", "warm", "cheerful",
    "tired", "sleepy", "confused", "concerned", "waiting"
]
```

### System Instructions

Core personality and behavior defined in `scripts/config/instructions.py`:

```python
DOG_COMPANION_INSTRUCTIONS = """
You are Rufus, a friendly robot dog companion in a home environment.
...
"""
```

## ROS2 Topics

### Published by Bridge

- **`/dog_voice_agent/status`** (`AgentStatus`): Real-time agent state and metrics
- **`/dog_voice_agent/tool_event`** (`ToolEvent`): Function tool invocation tracking
- **`/dog_voice_agent/vip_detection`** (`VipDetection`): Family member detection
- **`/dog_voice_agent/extension_event`** (`ExtensionEvent`): Conversation extensions

### Subscribed by Bridge

- **`/dog_voice_agent/virtual_request`** (`VirtualRequest`): External task requests
  - Types: `TASK_ASSIGNED`, `POOP_DETECTED`, `NAVIGATION_ARRIVED`, `PICKUP_SUCCESS`, etc.

## Task Event Flow

```
┌──────────────┐           ┌──────────────┐           ┌──────────────┐
│  Voice Agent │◄────────►│  ROS2 Bridge │◄────────►│  Robot Dog   │
│   (LiveKit)  │ WebSocket│   (WS Client)│  ROS2    │  (Hardware)  │
└──────────────┘           └──────────────┘           └──────────────┘

User Speech → STT → LLM → Tool Call → ROS2 Topic → Hardware Action
                                ↓
                         Tool Event Published
                                ↓
                    Status Update via WebSocket
                                ↓
                      Agent Announces Progress
```

## Development

### File-Based Modular Architecture

The refactored version separates concerns cleanly:

| **Module** | **File** | **Responsibility** |
|------------|----------|-------------------|
| Entry Point | `main.py` | Application startup and configuration |
| State Management | `state/state_manager.py` | Conversation state, timeouts, virtual requests |
| Agent Logic | `agents/dog_companion_agent.py` | I/O services, TTS, wake word, WebSocket |
| Tools | `tools/dog_tools.py` | Function implementations with event tracking |
| Configuration | `config/instructions.py`, `config/settings.py` | Personality and settings |
| Utilities | `utils/greeting_data.py`, `utils/announcement_data.py` | Data and templates |

### Adding New Capabilities

1. **Add Function Tool** (`tools/dog_tools.py`):
   ```python
   async def new_capability_impl(context: RunContext, arg: str) -> str:
       await send_tool_event("new_capability", "started", [arg])
       # ... implementation ...
       await send_tool_event("new_capability", "completed", [arg], result)
       return result
   ```

2. **Register Tool** (`agents/dog_companion_agent.py`):
   ```python
   function_tool(
       new_capability_impl,
       name="new_capability",
       description="Description for LLM"
   )
   ```

3. **Add Task Type** (`utils/announcement_data.py`):
   ```python
   "NEW_TASK_TYPE": "emotion:Response template {content}"
   ```

4. **Update Instructions** (`config/instructions.py`):
   Add capability description to `DOG_COMPANION_INSTRUCTIONS`

## Testing

### Console Mode Testing

Test the agent locally without ROS2:
```bash
cd scripts
./run_main.sh console
```

Interact via terminal:
- Type messages to test responses
- Test tool invocations
- Verify emotion parsing
- Test conversation timeouts

### Bridge Testing

1. Start voice agent in one terminal:
   ```bash
   ./run_main.sh
   ```

2. Start bridge in another terminal:
   ```bash
   ros2 launch dog_voice_agent voice_agent_bridge.launch.py
   ```

3. Monitor topics:
   ```bash
   ros2 topic echo /dog_voice_agent/status
   ros2 topic echo /dog_voice_agent/tool_event
   ```

4. Send virtual task requests:
   ```bash
   ros2 topic pub /dog_voice_agent/virtual_request \
     dog_voice_agent_msgs/VirtualRequest \
     "{type: 'POOP_DETECTED', content: 'in the corner', priority: 'normal'}"
   ```

## Troubleshooting

### Wake Word Not Working
- Verify `PORCUPINE_ACCESS_KEY` is set
- Check microphone permissions
- Try running without wake word (always-on mode)

### WebSocket Connection Failed
- Verify `WEBSOCKET_HOST` and `WEBSOCKET_PORT` match in both agent and bridge
- Check firewall settings
- Ensure voice agent starts before bridge

### LLM Not Responding
- Verify `OPENAI_API_KEY` is valid
- Check internet connectivity
- Review console logs for API errors

### Emotion Not Parsed
- Ensure responses follow `emotion:text` format
- Check emotion is in `VALID_EMOTIONS` list
- Verify TTS override is processing correctly

## Future Enhancements

- [ ] Add camera/vision integration for object detection
- [ ] Implement SLAM for better navigation
- [ ] Add multiple dog personality profiles
- [ ] Integrate with smart home systems
- [ ] Add voice command shortcuts for common tasks
- [ ] Implement multi-dog coordination
- [ ] Add learning from user preferences
- [ ] Create mobile app for remote control

## License

MIT

## Contributing

This is part of the LeRobot/LeDog project. Contributions welcome!

## Related Packages

- **`dog_voice_agent_msgs`**: Custom ROS2 message definitions
- **`lekiwi_base`**: Motor control for robot dog hardware
- **`lekiwi_bringup`**: Launch files and configuration
- **`lekiwi_description`**: URDF/XACRO robot model

## Credits

Based on the Coffee Voice Agent architecture, adapted for home robot dog companion use case.
