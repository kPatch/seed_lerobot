"""Robot dog companion function implementations for programmatic tool registration"""

import asyncio
import logging
from datetime import datetime
from livekit.agents import RunContext

logger = logging.getLogger(__name__)

# Global reference to the agent for tool event sending
_agent_instance = None

def set_agent_instance(agent):
    """Set the agent instance for tool event sending"""
    global _agent_instance
    _agent_instance = agent

async def send_tool_event(tool_name: str, status: str, parameters: list = None, result: str = ""):
    """Send tool event through the agent's state manager"""
    if _agent_instance and hasattr(_agent_instance, 'state_manager'):
        try:
            await _agent_instance.state_manager._send_tool_event(tool_name, status, parameters, result)
        except Exception as e:
            logger.error(f"Error sending tool event: {e}")
    else:
        logger.debug(f"Cannot send tool event - no agent instance available")


async def get_current_time_impl(context: RunContext) -> str:
    """Get the current time."""
    await send_tool_event("get_current_time", "started")
    
    current_time = datetime.now().strftime("%I:%M %p")
    result = f"The current time is {current_time}"
    logger.info(f"Time requested: {current_time}")
    
    await send_tool_event("get_current_time", "completed", [], result)
    return result


async def get_current_date_impl(context: RunContext) -> str:
    """Get today's date."""
    await send_tool_event("get_current_date", "started")
    
    current_date = datetime.now().strftime("%A, %B %d, %Y")
    result = f"Today's date is {current_date}"
    logger.info(f"Date requested: {current_date}")
    
    await send_tool_event("get_current_date", "completed", [], result)
    return result


async def get_dog_capabilities_impl(context: RunContext) -> str:
    """Get the robot dog's capabilities and features."""
    await send_tool_event("get_dog_capabilities", "started")
    
    capabilities = """🐕 ROBOT DOG CAPABILITIES

    🏃 MOBILITY:
    - Autonomous navigation
    - Follow people
    - Patrol areas
    - Return to charging station
    
    🎯 TASKS:
    - Pick up objects (including pet waste)
    - Monitor the home
    - Deliver items
    - Perform tricks
    
    🤖 AI FEATURES:
    - Natural conversation
    - Activity suggestions
    - Schedule reminders
    - Emotional companionship"""
    
    logger.info("Dog capabilities requested")
    
    await send_tool_event("get_dog_capabilities", "completed", [], capabilities)
    return capabilities


async def get_activity_suggestions_impl(context: RunContext) -> str:
    """Get suggestions for activities the robot dog can help with."""
    await send_tool_event("get_activity_suggestions", "started")
    
    suggestions = """🎯 ACTIVITY SUGGESTIONS:
    
    🏠 HOME TASKS:
    - Pick up pet waste from your other pets
    - Patrol the house while you're away
    - Monitor doors and windows
    - Deliver items between rooms
    
    🎮 FUN ACTIVITIES:
    - Play fetch (I'll bring items back!)
    - Perform tricks (sit, spin, play dead)
    - Follow you around the house
    - Keep you company
    
    📅 HELPFUL REMINDERS:
    - Schedule tracking
    - Time announcements
    - Activity suggestions based on time of day
    
    🤖 Just ask me to do any of these, and I'm happy to help!"""
    
    logger.info("Activity suggestions requested")
    
    await send_tool_event("get_activity_suggestions", "completed", [], suggestions)
    return suggestions


async def recommend_activity_impl(context: RunContext, preference: str = "active") -> str:
    """Recommend an activity based on user preference.
    
    Args:
        preference: Type of activity preference (active, relaxing, helpful, playful, etc.)
    """
    await send_tool_event("recommend_activity", "started", [preference])
    
    recommendations = {
        "active": "How about a patrol around the house? I can check all the rooms and make sure everything is secure! 🏃",
        "relaxing": "I can keep you company while you relax! I'll stay nearby and we can chat if you'd like. 😌",
        "helpful": "I can pick up any items that need moving, or help monitor the house. What would be most useful? 🤖",
        "playful": "Let's play! I can do tricks, play fetch, or follow you around. I'm always up for fun! 🎾",
        "productive": "I can help you stay on schedule by reminding you of tasks, or patrol while you work! 📋",
        "default": "I'd recommend a quick house patrol - I can check on things and make sure everything is in order! 🏠"
    }
    
    base_recommendation = recommendations.get(preference.lower(), recommendations["default"])
    
    full_recommendation = f"{base_recommendation}\n\n🐕 Just let me know what you'd like me to do!"
    
    logger.info(f"Activity recommendation for '{preference}': {base_recommendation}")
    
    await send_tool_event("recommend_activity", "completed", [preference], full_recommendation)
    return full_recommendation


async def manage_conversation_time_impl(
    context: RunContext,
    action: str,
    reason: str,
    user_importance: str = "normal",
    extension_minutes: int = 0
) -> str:
    """
    Intelligent conversation time management based on context.
    
    Args:
        action: What to do - "continue", "warn", "extend", "end"
        reason: LLM's reasoning for the decision
        user_importance: Assessment of user importance/engagement - "vip", "engaged", "normal"
        extension_minutes: How much to extend if action is 'extend'
    """
    await send_tool_event("manage_conversation_time", "started", [action, reason, user_importance])
    
    logger.info(f"LLM conversation decision: {action} - {reason} (user_importance: {user_importance})")
    
    result = ""
    
    if action == "end":
        # Signal the state manager to end conversation gracefully
        if _agent_instance and hasattr(_agent_instance, 'state_manager'):
            _agent_instance.state_manager.ending_conversation = True
        result = f"Conversation ending initiated: {reason}"
        
    elif action == "extend":
        # Actually extend the conversation by calling state manager
        if _agent_instance and hasattr(_agent_instance, 'state_manager') and extension_minutes > 0:
            await _agent_instance.state_manager.extend_conversation(extension_minutes, reason)
            
            # Send extension granted event via WebSocket
            if hasattr(_agent_instance, '_send_websocket_event'):
                await _agent_instance._send_websocket_event("EXTENSION_GRANTED", {
                    "action": "granted",
                    "extension_minutes": extension_minutes,
                    "reason": reason,
                    "granted_by": "tool",
                    "timestamp": datetime.now().isoformat()
                })
            
            logger.info(f"Conversation extended by {extension_minutes} minutes: {reason}")
            result = f"Conversation extended by {extension_minutes} minutes: {reason}"
        else:
            logger.warning(f"Cannot extend conversation - no agent instance or invalid extension minutes: {extension_minutes}")
            result = f"Extension failed: {reason}"
        
    elif action == "warn":
        logger.info(f"Conversation warning acknowledged: {reason}")
        result = f"Time warning acknowledged: {reason}"
        
    elif action == "continue":
        logger.info(f"Conversation continues normally: {reason}")
        result = f"Conversation continues: {reason}"
        
    else:
        result = f"Unknown action '{action}': {reason}"
        logger.warning(result)
    
    await send_tool_event("manage_conversation_time", "completed", [action, reason, user_importance], result)
    return result


async def check_user_status_impl(
    context: RunContext,
    user_identifier: str = ""
) -> str:
    """
    Check if user has special status (VIP, staff, important guest).
    
    Args:
        user_identifier: Any identifier mentioned by the user (name, title, etc.)
    """
    await send_tool_event("check_user_status", "started", [user_identifier])
    
    # VIP keywords to check for (family members, important guests)
    vip_keywords = [
        "mom", "dad", "family", "owner", "emergency", "doctor", 
        "caregiver", "important", "urgent", "guest"
    ]
    
    user_lower = user_identifier.lower()
    is_vip = any(keyword in user_lower for keyword in vip_keywords)
    
    if is_vip:
        # Set VIP session status - removes hard timeout, only inactivity timeout applies
        if _agent_instance and hasattr(_agent_instance, 'state_manager'):
            await _agent_instance.state_manager.set_vip_session(
                f"VIP user detected: {user_identifier}"
            )
            logger.info(f"Set VIP session for user: {user_identifier}")
        
        # Send VIP detection event via WebSocket
        if _agent_instance and hasattr(_agent_instance, '_send_websocket_event'):
            matched_keywords = [keyword for keyword in vip_keywords if keyword in user_lower]
            await _agent_instance._send_websocket_event("VIP_DETECTED", {
                "user_identifier": user_identifier,
                "matched_keywords": matched_keywords,
                "importance_level": "vip",
                "recommended_extension_minutes": 0,  # No hard timeout for VIP
                "timestamp": datetime.now().isoformat()
            })
            
            # Send VIP session granted event
            await _agent_instance._send_websocket_event("EXTENSION_GRANTED", {
                "action": "vip_session",
                "extension_minutes": 0,  # Unlimited until inactivity
                "reason": f"VIP user detected: {user_identifier}",
                "granted_by": "auto_vip_detection",
                "timestamp": datetime.now().isoformat()
            })
        
        result = f"VIP user detected: {user_identifier}. Hard timeout removed - conversation will continue until you become inactive. You can now provide unlimited personalized VIP service."
        logger.info(f"VIP user identified: {user_identifier}")
    else:
        result = f"Standard user: {user_identifier}. Normal time limits apply."
        logger.info(f"Standard user: {user_identifier}")
    
    await send_tool_event("check_user_status", "completed", [user_identifier], result)
    return result 