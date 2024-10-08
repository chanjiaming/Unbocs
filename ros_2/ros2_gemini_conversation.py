import socket
import os
import pygame
import speech_recognition as sr
from gtts import gTTS
import subprocess  # To relaunch hand_waving.py after the conversation
from chan_gemini_config import configure_gemini

generative_model = configure_gemini()

class GeminiChat:
    def __init__(self):
        self.user_name = None
        self.conversation_over = False  # Track when the user says goodbye
        pygame.init()
        print("Gemini Chat Initialized")

    def check_internet_connection(self):
        try:
            socket.create_connection(("8.8.8.8", 53), timeout=3)
            return True
        except OSError:
            return False

    def speak(self, text):
        """Speak the given text using gTTS and play it using pygame."""
        tts = gTTS(text=text, lang='en')
        audio_file = "temp_audio.mp3"
        tts.save(audio_file)
        pygame.mixer.music.load(audio_file)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            continue
        os.remove(audio_file)

    def generate_gemini_response(self, prompt):
        """Generate a response using the Gemini model."""
        if generative_model:
            try:
                response = generative_model.generate_content([prompt])
                if response and response.text:
                    return response.text.strip()
                else:
                    print("Gemini model returned no response.")
            except Exception as e:
                print(f"Failed to generate response using Gemini: {e}")
        else:
            print("Gemini model is not configured.")
        return None

    def listen(self):
        """Capture audio input and convert it to text using SpeechRecognition."""
        recognizer = sr.Recognizer()
        microphone = sr.Microphone()

        with microphone as source:
            print("Gemini: Listening for your response...")
            recognizer.adjust_for_ambient_noise(source)
            audio = recognizer.listen(source)

        try:
            print("Gemini: Recognizing speech...")
            user_input = recognizer.recognize_google(audio)
            print(f"You: {user_input}")
            return user_input
        except sr.UnknownValueError:
            print("Gemini: Sorry, I didn't catch that.")
            self.speak("Sorry, I didn't catch that. Could you please repeat?")
            return None
        except sr.RequestError:
            print("Gemini: Could not request results from the speech recognition service.")
            self.speak("I couldn't connect to the speech recognition service. Please try again.")
            return None

    def run(self):
        welcome_message = "Welcome! How can I assist you today?"
        self.speak(welcome_message)
        print("Gemini: Welcome! How can I assist you today?")

        while not self.conversation_over:
            user_input = self.listen()

            if user_input is None:
                continue

            if "goodbye" in user_input.lower():
                self.conversation_over = True
                farewell_message = "Goodbye! It was nice talking to you."
                print(f"Gemini: {farewell_message}")
                self.speak(farewell_message)
                break

            bot_response = self.generate_gemini_response(user_input)

            if bot_response:
                print(f"Gemini: {bot_response}")
                self.speak(bot_response)
            else:
                error_message = "I didn't quite understand that. Could you please repeat?"
                print(f"Gemini: {error_message}")
                self.speak(error_message)

        # After saying goodbye, restart the hand_waving.py
        self.initiate_hand_wave_detection()

    def initiate_hand_wave_detection(self):
        """Launch the hand_waving.py script after saying goodbye."""
        print("Restarting hand wave detection...")
        subprocess.Popen(["python3", "/path/to/hand_waving.py"])

if __name__ == "__main__":
    gemini_chat = GeminiChat()
    gemini_chat.run()
