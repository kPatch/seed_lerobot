"""State management for the robot dog companion agent

This module contains the StateManager class extracted from the original monolithic implementation.
It handles conversation state transitions, timeout management, and virtual request processing.
"""

import asyncio
import logging
import json
import os
from datetime import datetime
from enum import Enum

from livekit.agents import AgentSession
from livekit.plugins import openai, silero

from config.settings import (
    USER_RESPONSE_TIMEOUT, FINAL_TIMEOUT, MAX_CONVERSATION_TIME,
    CONVERSATION_WARNING_TIME, FINAL_WARNING_TIME, VALID_EMOTIONS
)

logger = logging.getLogger(__name__)


class AgentState(Enum):
    DORMANT = "dormant"           # Only wake word detection active
    CONNECTING = "connecting"     # Creating LiveKit session  
    ACTIVE = "active"            # Conversation mode
    SPEAKING = "speaking"        # Agent is speaking
    DISCONNECTING = "disconnecting" # Cleaning up session


class StateManager:
    def __init__(self, agent=None):
        self.current_state = AgentState.DORMANT
        self.state_lock = asyncio.Lock()
        self.session = None
        self.conversation_timer = None  # Max conversation time limit
        self.user_response_timer = None  # Timer for waiting for user response
        self.ctx = None
        self.agent = agent
        self.current_emotion = "waiting"  # Track current emotional state
        self.previous_emotion = ""  # Track previous emotion for smooth transitions
        self.emotion_history = []  # Log emotional journey
        self.ending_conversation = False  # Flag to prevent timer conflicts during goodbye
        
        # Text tracking for TTS events
        self.current_speech_preview = ""  # Preview text for "started" events
        self.current_speech_full_text = ""  # Accumulated full text for "finished" events
        
        # Tool tracking
        self.last_tool_used = ""  # Last function tool called
        self.virtual_request_queue = []  # Queue for virtual requests
        self.announcing_virtual_request = False  # Flag to prevent conflicts during announcements
        self.recent_greetings = []  # Track recent greetings to avoid repetition
        self.interaction_count = 0  # Track number of interactions for familiarity
        
        # Goodbye coordination flag
        self.goodbye_pending = False  # Flag to coordinate goodbye handling between user and assistant handlers
        self.end_after_current_speech = False  # Flag to end conversation when agent stops speaking
        
        # Conversation timing tracking for admin message injection
        self.conversation_start_time = None
        self.five_minute_warning_sent = False
        self.six_minute_warning_sent = False
        self.seven_minute_warning_sent = False
        self.extension_expired_pending = False
        self.is_vip_session = False
        
        # Phase 4: Mutual exclusion for virtual request processing
        self.virtual_request_processing_lock = asyncio.Lock()
        # Removed batching complexity - no longer needed
        # self.batch_timer = None
        # self.batch_window_duration = 2.0

    async def transition_to_state(self, new_state: AgentState):
        """Handle state transitions with proper cleanup"""
        async with self.state_lock:
            if self.current_state == new_state:
                return
                
            logger.info(f"State transition: {self.current_state.value} → {new_state.value}")
            await self._exit_current_state()
            self.current_state = new_state
            await self._enter_new_state()
            
            # Send agent status update for behavioral mode changes
            behavioral_mode_map = {
                AgentState.DORMANT: "dormant",
                AgentState.CONNECTING: "connecting", 
                AgentState.ACTIVE: "active",
                AgentState.DISCONNECTING: "disconnecting"
            }
            
            behavioral_mode = behavioral_mode_map.get(new_state, "dormant")
            
            # Determine conversation phase for greetings
            conversation_phase = ""
            if new_state == AgentState.ACTIVE:
                conversation_phase = "greeting"
            
            await self._send_agent_status(behavioral_mode, "idle", conversation_phase)

    async def _exit_current_state(self):
        """Clean up current state"""
        if self.current_state == AgentState.ACTIVE:
            # Cancel conversation timers
            if self.conversation_timer:
                self.conversation_timer.cancel()
                self.conversation_timer = None
            if self.user_response_timer:
                self.user_response_timer.cancel()
                self.user_response_timer = None
            # Reset conversation ending flag
            self.ending_conversation = False
            # Reset goodbye coordination flag
            self.goodbye_pending = False
        

    async def _enter_new_state(self):
        """Initialize new state"""
        if self.current_state == AgentState.ACTIVE:
            # Start conversation timing tracking
            import time
            self.conversation_start_time = time.time()
            self.five_minute_warning_sent = False
            self.six_minute_warning_sent = False
            self.seven_minute_warning_sent = False
            self.extension_expired_pending = False
            
            # Start max conversation timer (absolute limit)
            self.conversation_timer = asyncio.create_task(self._max_conversation_timeout())
        elif self.current_state == AgentState.DORMANT:
            # Reset conversation timing and VIP status
            self.conversation_start_time = None
            self.is_vip_session = False
            self.end_after_current_speech = False
            
            # Resume wake word detection when returning to dormant
            if self.agent:
                self.agent.wake_word_paused = False
                logger.info("Resumed wake word detection")

    async def _max_conversation_timeout(self):
        """Handle absolute maximum conversation time limit (fallback) - only for non-VIP sessions"""
        try:
            # Wait for absolute maximum time (7 minutes)
            await asyncio.sleep(MAX_CONVERSATION_TIME)
            if self.session and self.current_state == AgentState.ACTIVE and not self.is_vip_session:
                logger.info("Absolute maximum conversation time reached - ending conversation")
                
                # Set ending flag to prevent timer conflicts
                self.ending_conversation = True
                
                # Fallback timeout message if callback system didn't handle it
                timeout_response = "friendly:I've really enjoyed our conversation! I need to conserve energy now. Say 'hey rufus' if you need me again."
                emotion, text = self.process_emotional_response(timeout_response)
                await self.say_with_emotion(text, emotion)
                
                await asyncio.sleep(2)
                await self.end_conversation()
        except asyncio.CancelledError:
            pass

    async def extend_conversation(self, additional_minutes: int, reason: str):
        """Extend conversation by additional minutes, cancelling and restarting the timeout timer"""
        if self.current_state != AgentState.ACTIVE:
            logger.warning(f"Cannot extend conversation - not in active state (current: {self.current_state})")
            return
            
        logger.info(f"Extending conversation by {additional_minutes} minutes: {reason}")
        
        # Cancel existing conversation timer
        if self.conversation_timer:
            self.conversation_timer.cancel()
            self.conversation_timer = None
            
        # Reset conversation timing to start fresh from extension point
        import time
        self.conversation_start_time = time.time()
        self.five_minute_warning_sent = False
        self.six_minute_warning_sent = False
        self.seven_minute_warning_sent = False
        self.extension_expired_pending = False
        logger.info("Reset conversation timing and warning flags due to extension")
            
        # Calculate new timeout duration (additional time from now)
        new_timeout_seconds = additional_minutes * 60
        
        # Start new conversation timer with extended duration
        async def extended_timeout():
            try:
                await asyncio.sleep(new_timeout_seconds)
                if self.session and self.current_state == AgentState.ACTIVE:
                    logger.info(f"Extended conversation time reached - flagging for admin message injection")
                    
                    # Set flag for admin message injection on next user turn
                    self.extension_expired_pending = True
                    
                    # Fallback timer - if user doesn't speak within 30 seconds, end with hardcoded message
                    await asyncio.sleep(30)
                    if self.session and self.current_state == AgentState.ACTIVE and self.extension_expired_pending:
                        logger.info("Extension expired and no user response - ending with fallback message")
                        
                        # Set ending flag to prevent timer conflicts
                        self.ending_conversation = True
                        
                        # Fallback ending message
                        timeout_response = "friendly:I've really enjoyed our extended conversation! I need to conserve energy now. Say 'hey rufus' if you need me again."
                        emotion, text = self.process_emotional_response(timeout_response)
                        await self.say_with_emotion(text, emotion)
                        
                        await asyncio.sleep(2)
                        await self.end_conversation()
            except asyncio.CancelledError:
                pass
                
        self.conversation_timer = asyncio.create_task(extended_timeout())

    async def set_vip_session(self, reason: str):
        """Set VIP session status and remove hard timeout - only inactivity timeout applies"""
        logger.info(f"Setting VIP session status: {reason}")
        
        # Mark as VIP session
        self.is_vip_session = True
        
        # Cancel existing hard timeout permanently for VIP users
        if self.conversation_timer:
            self.conversation_timer.cancel()
            self.conversation_timer = None
            logger.info("Cancelled hard timeout for VIP session - only inactivity timeout applies")

    async def _wait_for_user_response(self):
        """Wait for user response after agent speaks"""
        try:
            # Wait for user to respond, but check for virtual requests during pause
            pause_duration = 0
            check_interval = 2  # Check every 2 seconds
            
            while pause_duration < USER_RESPONSE_TIMEOUT:
                await asyncio.sleep(check_interval)
                pause_duration += check_interval
                
                # Check for virtual requests during the pause
                if self.virtual_request_queue and not self.announcing_virtual_request:
                    # Process virtual request during conversation pause
                    await self._process_virtual_request_during_conversation()
                    # Reset pause duration after announcement
                    pause_duration = 0
            
            if self.session and self.current_state == AgentState.ACTIVE:
                # Polite prompt with curious emotion

                prompt = "curious:Is there anything else I can help you with?"
                emotion, text = self.process_emotional_response(prompt)
                await self.say_with_emotion(text, emotion)
                
                # Wait a bit more
                await asyncio.sleep(FINAL_TIMEOUT)
                
                if self.session and self.current_state == AgentState.ACTIVE:
                    # Set ending flag to prevent timer conflicts
                    self.ending_conversation = True
                    
                    # End conversation with friendly emotion using delimiter format
                    goodbye_response = "friendly:Thanks for chatting! Say 'hey rufus' if you need me again."
                    emotion, text = self.process_emotional_response(goodbye_response)
                    await self.say_with_emotion(text, emotion)
                    
                    await asyncio.sleep(2)
                    await self.end_conversation()
                
        except asyncio.CancelledError:
            pass  # User spoke, timer cancelled

    async def create_session(self, agent, for_conversation: bool = True) -> AgentSession:
        """Create new session for voice interaction
        
        Args:
            agent: The voice agent instance
            for_conversation: If True, adds conversation event handlers for interactive sessions.
                            If False, creates a minimal session for announcements only.
                            This prevents race conditions during batch announcement processing.
        """
        if self.session:
            await self.destroy_session()
            
        self.session = AgentSession(
            stt=openai.STT(model="whisper-1"),
            llm=openai.LLM(
                model="gpt-4o-mini",
                temperature=float(os.getenv("VOICE_AGENT_TEMPERATURE", "0.7"))
            ),
            tts=openai.TTS(
                model="tts-1",
                voice=os.getenv("VOICE_AGENT_VOICE", "nova")
            ),
            vad=silero.VAD.load(),
        )
        
        # Set up session event handlers using correct LiveKit 1.0+ events
        # Only add conversation handlers for interactive sessions to prevent race conditions
        # during DORMANT state batch announcement processing
        if for_conversation:
            @self.session.on("conversation_item_added")
            def on_conversation_item_added(event):
                """Handle conversation items being added - detect user goodbye and manage conversation flow"""
                logger.info("🔍 DEBUG: conversation_item_added event fired!")
                logger.info(f"🔍 DEBUG: event type: {type(event)}")
                logger.info(f"🔍 DEBUG: item role: {event.item.role}")
                logger.info(f"🔍 DEBUG: item text: {event.item.text_content}")
                
                async def handle_conversation_item():
                    try:
                        # Handle user messages for goodbye detection
                        if event.item.role == "user":
                            # Cancel user response timer when user speaks
                            if self.user_response_timer:
                                self.user_response_timer.cancel()
                                self.user_response_timer = None
                            
                            # Check for goodbye in user text
                            user_text = event.item.text_content or ""
                            text_lower = user_text.lower()
                            logger.info(f"🔍 DEBUG: User said: '{user_text}'")
                            
                            # Send user speech event
                            await self._send_user_speech_event(user_text)
                            
                            goodbye_words = ['goodbye', 'thanks', 'that\'s all', 'see you', 'bye']
                            
                            if any(word in text_lower for word in goodbye_words):
                                logger.info("🔍 DEBUG: User indicated conversation ending - goodbye detected!")
                                # Set pending flag to coordinate with LLM response
                                self.goodbye_pending = True
                                logger.info("🔍 DEBUG: Set goodbye_pending = True, letting LLM respond naturally")
                            else:
                                logger.info(f"🔍 DEBUG: No goodbye detected in: '{user_text}'")
                        
                        # Handle agent messages for timer management
                        elif event.item.role == "assistant":
                            logger.info("🔍 DEBUG: Agent message added to conversation")
                            
                            # Check if this is a response to a user goodbye
                            if self.goodbye_pending:
                                logger.info("🔍 DEBUG: LLM responded to user goodbye - ending conversation")
                                self.goodbye_pending = False
                                self.ending_conversation = True
                                
                                # Set flag to end conversation when TTS completes
                                self.end_after_current_speech = True
                                return  # Skip normal timer logic since we're ending
                            
                            # Check if this is a system-initiated ending
                            if self.ending_conversation:
                                logger.info("🔍 DEBUG: System-initiated ending - will end after TTS completes")
                                
                                # Set flag to end conversation when TTS completes
                                self.end_after_current_speech = True
                                return  # Skip normal timer logic since we're ending
                            
                            # Only start new timer if we're not ending the conversation
                            if not self.ending_conversation:
                                # Start timer to wait for user response after agent speaks
                                if self.user_response_timer:
                                    self.user_response_timer.cancel()
                                
                                self.user_response_timer = asyncio.create_task(self._wait_for_user_response())
                                logger.info("🔍 DEBUG: Started user response timer")
                            else:
                                logger.info("🔍 DEBUG: Skipping user response timer - conversation ending")
                                
                    except Exception as e:
                        logger.error(f"🔍 DEBUG: Exception in conversation_item_added handler: {e}")
                        import traceback
                        logger.error(f"🔍 DEBUG: Traceback: {traceback.format_exc()}")
                        
                asyncio.create_task(handle_conversation_item())
            
            @self.session.on("user_state_changed")
            def on_user_state_changed(event):
                """Handle user state changes (speaking/listening)"""
                logger.info(f"🔍 DEBUG: user_state_changed: {event.old_state} → {event.new_state}")
                
                if event.new_state == "speaking":
                    logger.info("🔍 DEBUG: User started speaking")
                elif event.new_state == "listening":
                    logger.info("🔍 DEBUG: User stopped speaking")
            
            @self.session.on("agent_state_changed")
            def on_agent_state_changed(event):
                """Handle agent state changes (initializing/listening/thinking/speaking)"""
                logger.info(f"🔍 DEBUG: agent_state_changed: {event.old_state} → {event.new_state}")
                
                # Send unified agent status based on state transitions
                async def handle_state_change():
                    try:
                        # Map LiveKit agent states to our behavioral modes
                        current_behavioral_mode = "dormant"  # Default
                        if self.current_state == AgentState.ACTIVE:
                            current_behavioral_mode = "active"
                        elif self.current_state == AgentState.CONNECTING:
                            current_behavioral_mode = "connecting"
                        elif self.current_state == AgentState.DISCONNECTING:
                            current_behavioral_mode = "disconnecting"
                        
                        if event.new_state == "speaking":
                            logger.info("🔍 DEBUG: Agent started speaking - sending agent status")
                            await self._send_agent_status(current_behavioral_mode, "speaking")
                        elif event.old_state == "speaking" and event.new_state != "speaking":
                            logger.info("🔍 DEBUG: Agent stopped speaking - sending agent status")
                            await self._send_agent_status(current_behavioral_mode, "idle")
                            
                            # Check if we should end conversation after TTS completion
                            if self.end_after_current_speech:
                                logger.info("🔍 DEBUG: TTS completed - ending conversation as requested")
                                self.end_after_current_speech = False
                                asyncio.create_task(self.end_conversation())
                    except Exception as e:
                        logger.error(f"Error handling agent state change status events: {e}")
                
                asyncio.create_task(handle_state_change())
                
                if event.new_state == "speaking":
                    logger.info("🔍 DEBUG: Agent started speaking")
                elif event.new_state == "listening":
                    logger.info("🔍 DEBUG: Agent is listening")
                elif event.new_state == "thinking":
                    logger.info("🔍 DEBUG: Agent is thinking")
            
            @self.session.on("close")
            def on_session_close(event):
                """Handle session close events"""
                logger.info("🔍 DEBUG: session close event fired!")
                if event.error:
                    logger.error(f"🔍 DEBUG: Session closed with error: {event.error}")
                else:
                    logger.info("🔍 DEBUG: Session closed normally")
        
        await self.session.start(agent=agent, room=self.ctx.room)
        
        # Debug session after creation
        logger.info(f"🔍 DEBUG: Session created: {self.session}")
        logger.info(f"🔍 DEBUG: Session type: {type(self.session)}")
        
        return self.session

    async def destroy_session(self):
        """Clean up session when conversation ends"""
        if self.session:
            try:
                await self.session.aclose()
            except Exception as e:
                logger.error(f"Error closing session: {e}")
            finally:
                self.session = None

    async def end_conversation(self):
        """End the current conversation and return to dormant state"""
        logger.info("🔍 DEBUG: end_conversation called - cleaning up and transitioning to dormant")
        await self.destroy_session()
        await self.transition_to_state(AgentState.DORMANT)
        
        # Process any queued virtual requests when conversation ends
        await self._process_queued_virtual_requests()
        
        logger.info("🔍 DEBUG: end_conversation completed - now in dormant state")

    async def queue_virtual_request(self, request_type: str, content: str, priority: str = "normal"):
        """Add a virtual request to the queue with batching support"""
        request = {
            "type": request_type,
            "content": content,
            "priority": priority,
            "timestamp": datetime.now()
        }
        
        # Insert based on priority (urgent requests go to front)
        if priority == "urgent":
            self.virtual_request_queue.insert(0, request)
        else:
            self.virtual_request_queue.append(request)
            
        logger.info(f"📋 Queued virtual request: {request_type} - {content} (priority: {priority}) [Queue size: {len(self.virtual_request_queue)}]")
        
        # If agent is in DORMANT state, process requests immediately
        if self.current_state == AgentState.DORMANT:
            await self._process_virtual_requests()
        else:
            logger.info(f"🔍 Agent in {self.current_state.value} state - request will be processed during conversation")



    async def _process_virtual_requests(self):
        """Process all queued virtual requests with simple while loop - processes until queue is empty"""
        # Check if we're already processing (mutual exclusion)
        if self.virtual_request_processing_lock.locked():
            logger.info("🔍 Virtual request processing already in progress - skipping")
            return
        
        async with self.virtual_request_processing_lock:
            # Create session if needed
            temp_session = None
            try:
                if not self.session:
                    temp_session = await self.create_session(self.agent, for_conversation=False)
                    logger.info("🔍 Created temporary session for virtual request processing")
                
                # Process all requests until queue is empty
                processed_count = 0
                while self.virtual_request_queue and self.current_state == AgentState.DORMANT:
                    try:
                        # Simple dequeue from front
                        request = self.virtual_request_queue.pop(0)
                        processed_count += 1
                        
                        # Announce the virtual request
                        announcement = self._format_virtual_request_announcement(request)
                        emotion, text = self.process_emotional_response(announcement)
                        await self.say_with_emotion(text, emotion)
                        
                        logger.info(f"📢 Announced virtual request {processed_count}: {request['type']} [Queue size now: {len(self.virtual_request_queue)}]")
                        
                        # Small delay between announcements
                        if self.virtual_request_queue:
                            await asyncio.sleep(1.0)
                        
                    except Exception as e:
                        logger.error(f"Error processing virtual request {request.get('type', 'UNKNOWN') if 'request' in locals() else 'UNKNOWN'}: {e}")
                
                if processed_count > 0:
                    logger.info(f"🔍 Virtual request processing completed - processed {processed_count} requests")
                    await asyncio.sleep(1.0)  # Final pause
                
            except Exception as e:
                logger.error(f"Error in virtual request processing: {e}")
            finally:
                # Clean up temporary session
                if temp_session and self.session:
                    await self.destroy_session()
                    logger.info("🔍 Cleaned up temporary virtual request processing session")

    async def _process_virtual_request_during_conversation(self):
        """Process a virtual request during conversation pause"""
        if not self.virtual_request_queue or self.announcing_virtual_request:
            return
            
        self.announcing_virtual_request = True
        
        try:
            request = self.virtual_request_queue.pop(0)
            
            # Brief polite interruption
            # excuse_msg = "excuse:Oh, excuse me one moment..."
            # emotion, text = self.process_emotional_response(excuse_msg)
            # await self.say_with_emotion(text, emotion)
            
            # await asyncio.sleep(5)
            
            # Announce the virtual request
            announcement = self._format_virtual_request_announcement(request)
            emotion, text = self.process_emotional_response(announcement)
            await self.say_with_emotion(text, emotion)
            
            await asyncio.sleep(1)
            
            # # Resume conversation
            # resume_msg = "friendly:Now, where were we?"
            # emotion, text = self.process_emotional_response(resume_msg)
            # await self.say_with_emotion(text, emotion)
            
            logger.info(f"📢 Announced virtual request during conversation: {request['type']}")
            
        except Exception as e:
            logger.error(f"Error processing virtual request during conversation: {e}")
        finally:
            self.announcing_virtual_request = False

    async def _process_queued_virtual_requests(self):
        """Legacy method - now delegates to simplified processing system"""
        logger.info("🔍 Legacy _process_queued_virtual_requests called - delegating to simplified processing")
        await self._process_virtual_requests()

    def _format_virtual_request_announcement(self, request: dict) -> str:
        """Format virtual request as emotional announcement"""
        from utils.announcement_data import format_virtual_request_announcement
        
        return format_virtual_request_announcement(request)

    def process_emotional_response(self, llm_response: str) -> tuple[str, str]:
        """Process LLM response and extract emotion + text from delimiter format (emotion:text)"""
        try:
            # Check if response uses delimiter format (emotion:text)
            if ":" in llm_response:
                # Split on first colon
                parts = llm_response.split(":", 1)
                emotion = parts[0].strip()
                text = parts[1].strip() if len(parts) > 1 else ""
                
                logger.info(f"🔍 DEBUG: Delimiter format detected - emotion: '{emotion}', text: '{text[:50]}{'...' if len(text) > 50 else ''}'")
                
            else:
                # Try to parse as JSON (legacy format)
                try:
                    response_data = json.loads(llm_response)
                    emotion = response_data.get("emotion", "friendly")
                    text = response_data.get("text", "")
                    logger.info("🔍 DEBUG: JSON format detected (legacy)")
                except json.JSONDecodeError:
                    # Fallback: treat entire response as text with default emotion
                    logger.warning("No delimiter or JSON format found, using fallback")
                    emotion = "friendly"
                    text = llm_response
            
            # Validate emotion is in our supported set
            if emotion not in VALID_EMOTIONS:
                logger.warning(f"Unknown emotion '{emotion}', defaulting to 'friendly'")
                emotion = "friendly"
            
            # Log emotion transition
            if emotion != self.current_emotion:
                logger.info(f"🎭 Emotion transition: {self.current_emotion} → {emotion}")
                self.log_animated_eyes(emotion)
                self.previous_emotion = self.current_emotion  # Store previous before updating
                self.current_emotion = emotion
                
                # Store in emotion history
                self.emotion_history.append({
                    'timestamp': datetime.now(),
                    'emotion': emotion,
                    'text_preview': text[:50] + "..." if len(text) > 50 else text
                })
            
            return emotion, text
            
        except Exception as e:
            logger.error(f"Error processing emotional response: {e}")
            return "friendly", llm_response

    def log_animated_eyes(self, emotion: str):
        """Log how this emotion would appear as animated eyes"""
        from utils.animation_data import log_animated_eyes
        
        log_animated_eyes(emotion)

    async def say_with_emotion(self, text: str, emotion: str = None):
        """Speak text and log emotional context"""
        logger.info("🔍 DEBUG: say_with_emotion called - MANUAL TTS PATHWAY")
        logger.info(f"🔍 DEBUG: say_with_emotion text: '{text[:100]}{'...' if len(text) > 100 else ''}'")
        logger.info(f"🔍 DEBUG: say_with_emotion emotion: {emotion}")
        
        if self.session:
            # Store text for TTS events
            self.current_speech_preview = text[:50] + "..." if len(text) > 50 else text
            self.current_speech_full_text = text
            
            # Send TTS_STARTED event - COMMENTED OUT to prevent duplicates (using agent_state_changed instead)
            # logger.info("🔍 DEBUG: About to send TTS_STARTED event")
            # await self._send_tts_event("started", text, emotion or self.current_emotion, "manual")
            # logger.info("🔍 DEBUG: TTS_STARTED event sent successfully")
            
            logger.info("🔍 DEBUG: Calling session.say() directly (bypasses llm_node)")
            handle = await self.session.say(text)
            logger.info(f"🔍 DEBUG: session.say() returned handle: {type(handle)}")
            
            # Wait for TTS completion
            logger.info("🔍 DEBUG: About to call handle.wait_for_playout() - this might hang!")
            try:
                await handle.wait_for_playout()
                logger.info("🔍 DEBUG: handle.wait_for_playout() completed successfully!")
            except Exception as e:
                logger.error(f"🔍 DEBUG: handle.wait_for_playout() failed: {e}")
                raise
            
            # Send TTS_FINISHED event - COMMENTED OUT to prevent duplicates (using agent_state_changed instead)
            # logger.info("🔍 DEBUG: About to send TTS_FINISHED event")
            # await self._send_tts_event("finished", text, emotion or self.current_emotion, "manual")
            # logger.info("🔍 DEBUG: TTS_FINISHED event sent successfully")
            
            if emotion:
                logger.info(f"🎭 Speaking with emotion: {emotion}")
                logger.info(f"💬 Text: {text[:100]}{'...' if len(text) > 100 else ''}")
        else:
            logger.error("🔍 DEBUG: No session available for speaking")

    def get_random_greeting(self) -> str:
        """Get a random greeting from the greeting pool"""
        from utils.greeting_data import get_random_greeting
        
        return get_random_greeting()

    async def _send_agent_status(self, behavioral_mode: str, speech_status: str, conversation_phase: str = ""):
        """Send unified agent status through agent's WebSocket connection"""
        if self.agent and hasattr(self.agent, '_send_websocket_event'):
            # Determine conversation phase if not provided
            if not conversation_phase:
                if self.announcing_virtual_request:
                    conversation_phase = "announcement"
                elif behavioral_mode == "active":
                    conversation_phase = "discussion"
                # else conversation_phase remains empty for dormant
            
            # Get current speech text based on speech status
            speech_text = ""
            if speech_status == "speaking":
                speech_text = self.current_speech_preview or ""
            elif speech_status == "idle" and self.current_speech_full_text:
                speech_text = self.current_speech_full_text
            
            status_data = {
                "behavioral_mode": behavioral_mode,
                "speech_status": speech_status,
                "emotion": self.current_emotion,
                "speech_text": speech_text,
                "previous_emotion": getattr(self, 'previous_emotion', ''),
                "conversation_phase": conversation_phase,
                "last_tool_used": self.last_tool_used,
                "timestamp": datetime.now().isoformat()
            }
            await self.agent._send_websocket_event("AGENT_STATUS", status_data)
        else:
            logger.debug(f"Cannot send agent status - no agent WebSocket connection")

    async def _send_tool_event(self, tool_name: str, status: str, parameters: list = None, result: str = ""):
        """Send tool event through agent's WebSocket connection"""
        if self.agent and hasattr(self.agent, '_send_websocket_event'):
            # Update last tool used when tool starts
            if status == "started":
                self.last_tool_used = tool_name
            
            tool_data = {
                "tool_name": tool_name,
                "status": status,
                "parameters": parameters or [],
                "result": result,
                "timestamp": datetime.now().isoformat()
            }
            await self.agent._send_websocket_event("TOOL_EVENT", tool_data)
        else:
            logger.debug(f"Cannot send tool event - no agent WebSocket connection")

    async def _send_user_speech_event(self, text: str):
        """Send user speech event through agent's WebSocket connection"""
        if self.agent and hasattr(self.agent, '_send_websocket_event'):
            speech_data = {
                "text": text,
                "timestamp": datetime.now().isoformat()
            }
            await self.agent._send_websocket_event("USER_SPEECH", speech_data)
        else:
            logger.debug(f"Cannot send user speech event - no agent WebSocket connection")

    def get_configuration_dict(self):
        """
        Get all configuration values for sharing with bridge and UI components.
        
        This method provides a single source of truth for configuration that can be
        sent via WebSocket to the bridge and propagated to UI components as ROS2 parameters.
        
        Returns:
            dict: Complete configuration including timeouts, emotions, and WebSocket settings
        """
        logger.info("🔧 DEBUG: get_configuration_dict() called")
        
        from config.settings import WEBSOCKET_HOST, WEBSOCKET_PORT
        
        config = {
            # Core timing configuration
            "user_response_timeout": USER_RESPONSE_TIMEOUT,
            "final_timeout": FINAL_TIMEOUT,
            "max_conversation_time": MAX_CONVERSATION_TIME,
            
            # Emotion configuration
            "valid_emotions": list(VALID_EMOTIONS),
            
            # WebSocket configuration (for bridge connectivity info)
            "websocket_host": WEBSOCKET_HOST,
            "websocket_port": WEBSOCKET_PORT,
            
            # Version and metadata
            "agent_version": "refactored",
            "config_timestamp": datetime.now().isoformat()
        }
        
        logger.info(f"🔧 DEBUG: Generated config dict: {config}")
        return config 