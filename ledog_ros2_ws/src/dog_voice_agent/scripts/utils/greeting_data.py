"""Greeting data constants for the robot dog companion agent"""

import random
import logging

logger = logging.getLogger(__name__)

# Pool of greeting messages for the robot dog companion
GREETING_POOL = [
    "playful:Woof woof! Hey there! I'm Rufus, your robot dog companion! What can I help you with today?",
    "excited:Hi! I'm so happy to see you! Need any help around the house?",
    "friendly:Hello! I'm Rufus, your friendly home helper! What should we do today?",
    "cheerful:Hey! Ready to help with whatever you need! Want me to patrol the house?",
    "warm:Welcome home! I've been waiting for you! Anything I can help with?",
    "curious:Oh hi! What are we doing today? I'm ready for anything!",
    "enthusiastic:Hey! I'm charged up and ready to help! What's on the agenda?",
    "happy:Great to see you! I can help with cleaning, monitoring, or just keep you company!",
    "attentive:Hello! I'm here and ready to assist. What do you need?",
    "playful:Woof! Time for some fun or chores? I'm good with either!",
    "helpful:Hi there! Your loyal robot dog at your service! What task can I help with?",
    "excited:Hey! I've been practicing my tricks! Want to see, or should we get to work?",
    "warm:Welcome! I'm all charged up and ready to be useful! How can I help today?",
    "friendly:Hey! Great timing! I was just thinking about what we could do together!",
    "cheerful:Hello! The house is quiet and I'm ready for action! What do you need?",
]


def get_random_greeting() -> str:
    """Get a random greeting from the greeting pool"""
    selected_greeting = random.choice(GREETING_POOL)
    logger.info(f"🎭 Selected random greeting: {selected_greeting[:50]}...")
    return selected_greeting
