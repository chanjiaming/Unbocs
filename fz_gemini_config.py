#!/home/eevee/catkin_ws/src/fz_gemini/.venv/bin/python3

import google.generativeai as genai
import rospy
from config import API_KEY

def configure_gemini():
    try:
        genai.configure(api_key=API_KEY)
        generation_config = {
            "temperature": 0.5,
            "top_p": 0.95,
            "top_k": 64,
            "max_output_tokens": 200,
            "response_mime_type": "text/plain",
        }

        safety_settings = [
            {
                "category": "HARM_CATEGORY_HARASSMENT",
                "threshold": "BLOCK_NONE",
            },
            {
                "category": "HARM_CATEGORY_HATE_SPEECH",
                "threshold": "BLOCK_ONLY_HIGH",
            },
            {
                "category": "HARM_CATEGORY_SEXUALLY_EXPLICIT",
                "threshold": "BLOCK_ONLY_HIGH",
            },
            {
                "category": "HARM_CATEGORY_DANGEROUS_CONTENT",
                "threshold": "BLOCK_ONLY_HIGH",
            },
        ]

        generative_model = genai.GenerativeModel(
            model_name="gemini-1.5-flash",
            safety_settings=safety_settings,
            generation_config=generation_config,
            system_instruction="""You will be receiving images of a person which is the guest. 
                                I want you to describe the person by stating his/her gender and age, preferably with a number. 
                                Then, describe the person's attire, especially the color of their clothing. 
                                MAKE SURE to include description of glasses and masks, if they are not wearing any, explicitly say so.
                                Then, provide the guest's location, which will be given explicitly and then add on any other notable things surrounding them.
                                This will repeat multiple times for different guests.
                                The description of each guest should be unique; i.e., not shared with any other guest. 
                                This may include gender, age, glass, mask, clothing, objects surrounding them etc.
                                Make the response short to around 20 words""",
        )

        return generative_model

    except Exception as e:
        rospy.logerr("Failed to configure Generative AI: %s", e)
        return None
