import socket
import os
import pygame
from gtts import gTTS
from chan_gemini_config import configure_gemini

# Configure the Google Generative AI
generative_model = configure_gemini()

class GeminiChat:
    def __init__(self):
        self.user_name = None
        self.conversation_over = False  # Track when the user says goodbye
        pygame.init()  # Initialize Pygame
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
        audio_file = "temp_audio.mp3"  # Temporary audio file

        # Save the audio file
        tts.save(audio_file)

        # Load and play the audio file
        pygame.mixer.music.load(audio_file)
        pygame.mixer.music.play()

        # Wait until the audio finishes playing
        while pygame.mixer.music.get_busy():
            continue

        # Remove the temporary audio file
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

    def run(self):
        # Welcome message
        welcome_message = "Welcome! How can I assist you today?"
        self.speak(welcome_message)
        print("Gemini: Welcome! How can I assist you today?")

        while not self.conversation_over:
            user_input = input("You: ")  # Get user input from the terminal

            if "goodbye" in user_input.lower():
                self.conversation_over = True
                farewell_message = "Goodbye! It was nice talking to you."
                print(f"Gemini: {farewell_message}")
                self.speak(farewell_message)
                break

            # Generate response using Gemini model
            bot_response = self.generate_gemini_response(user_input)

            if bot_response:
                print(f"Gemini: {bot_response}")
                self.speak(bot_response)
            else:
                error_message = "I didn't quite understand that. Could you please repeat?"
                print(f"Gemini: {error_message}")
                self.speak(error_message)

if __name__ == "__main__":
    gemini_chat = GeminiChat()
    gemini_chat.run()
