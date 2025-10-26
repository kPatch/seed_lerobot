"""Announcement templates for virtual task requests and events"""

# Templates for formatting virtual request announcements from ROS2 events
REQUEST_ANNOUNCEMENT_TEMPLATES = {
    # Task-related announcements
    "TASK_ASSIGNED": "focused:Got it! I'll {content}. On my way!",
    "TASK_COMPLETE": "proud:All done! I finished {content}!",
    "TASK_FAILED": "confused:Sorry, I couldn't complete {content}. Something went wrong.",
    
    # Poop cleanup specific (the signature feature!)
    "POOP_DETECTED": "alert:Heads up! I detected some poop {content}. Should I go pick it up?",
    "POOP_CLEANUP_STARTED": "focused:On my way to pick up the mess {content}!",
    "POOP_PICKUP_SUCCESS": "proud:Mission accomplished! I picked up {content}!",
    "POOP_PICKUP_FAILED": "confused:Hmm, I'm having trouble picking up {content}. Need some help!",
    
    # Navigation events
    "NAVIGATION_STARTED": "focused:Heading to {content} now!",
    "NAVIGATION_ARRIVED": "happy:I've arrived at {content}!",
    "NAVIGATION_BLOCKED": "concerned:Oops! I can't get to {content}. Something is blocking my path.",
    "NAVIGATION_FAILED": "confused:Sorry, I couldn't navigate to {content}. Let me try another route.",
    
    # Battery and maintenance
    "LOW_BATTERY": "tired:I'm running low on battery. Heading to my charging station at {content}!",
    "CHARGING_STARTED": "sleepy:Time for a power nap! Charging at {content}.",
    "CHARGING_COMPLETE": "excited:All charged up and ready to go! Battery at {content}!",
    
    # Home monitoring
    "DOOR_OPEN": "alert:Hey! The {content} door is open. Just letting you know!",
    "DOOR_CLOSED": "friendly:The {content} door is now closed.",
    "WINDOW_OPEN": "alert:The {content} window is open. Want me to check it out?",
    "MOTION_DETECTED": "alert:I detected motion {content}. Checking it out now!",
    "UNUSUAL_SOUND": "curious:I heard something unusual {content}. Should I investigate?",
    
    # Delivery tasks
    "DELIVERY_STARTED": "helpful:Taking {content} to its destination!",
    "DELIVERY_COMPLETE": "proud:Delivered {content} successfully!",
    "ITEM_RETRIEVED": "happy:Found {content}! Bringing it to you now!",
    
    # Patrol and monitoring
    "PATROL_STARTED": "alert:Starting patrol of {content}. I'll report back soon!",
    "PATROL_COMPLETE": "cheerful:Patrol of {content} complete! Everything looks good!",
    "PATROL_ISSUE": "concerned:Found something during patrol {content}. You might want to check this out.",
    
    # Schedule and reminders
    "REMINDER": "helpful:Reminder: {content}",
    "SCHEDULE_UPDATE": "friendly:Just a heads up: {content}",
    
    # Generic events
    "NOTIFICATION": "friendly:Notification: {content}",
    "UPDATE": "helpful:Update: {content}",
    "DEFAULT": "friendly:{content}"
}


def format_virtual_request_announcement(request: dict) -> str:
    """Format virtual request as emotional announcement"""
    request_type = request["type"]
    content = request["content"]
    
    # Get template from dictionary, default to "DEFAULT" if type not found
    template = REQUEST_ANNOUNCEMENT_TEMPLATES.get(request_type, REQUEST_ANNOUNCEMENT_TEMPLATES["DEFAULT"])
    return template.format(content=content)
