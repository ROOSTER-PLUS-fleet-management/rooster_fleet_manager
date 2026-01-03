#!/usr/bin/env python
# This script works on ROS Melodic (Python 2.7)

import rospy
import rospkg
from std_msgs.msg import String, Bool
import os
import json
from num2word import word as n2w
import re
from pocketsphinx import LiveSpeech


NODE_NAME = "[voice_command]"

# --- CONFIGURATION ---
# File paths
JSGF_FILE = "/config/voice_command.jsgf"
DICT_FILE = '/config/voice_command.dict' 
MASTER_DICT = "/config/cmudict-0.7b.txt"
LOCATIONS_JSON_FILE = "/scripts/JSONtoRosparam/locations.JSON"
# URL to the standard CMU Dictionary (130,000 words) -> "http://svn.code.sf.net/p/cmusphinx/code/trunk/cmudict/cmudict-0.7b"

# Static commands definition
move_actions = ["move"]
transport_actions = ["transport"]
handling_actions = ["load", "empty"]
order_keywords = move_actions + transport_actions + handling_actions

simple_commands = ["place order", "stop listening", "clear list", "cancel order"]
priority_keywords = ["high", "medium", "low"]
misc_words = ["priority", "to"]

# Combined, deduped, normalized (lowercase) word list
basic_word_list = sorted({
    w.strip().lower()
    for phrase in (order_keywords + simple_commands + priority_keywords + misc_words)
    for w in phrase.split()
})

class VoiceCommandNode:
    def __init__(self):
        rospy.init_node(NODE_NAME.strip("[] "))
        self.rfm_path = rospkg.RosPack().get_path("rooster_fleet_manager")
        self.mrb_path = rospkg.RosPack().get_path("multi_robot_sim")
        
        # Publisher for the commands
        self.command_publisher = rospy.Publisher('/voice_command/output', String, queue_size=10)
        self.error_publisher = rospy.Publisher('/voice_command/error', String, queue_size=10)

        # Subscriber to the UI button state
        self.is_recording = False
        rospy.Subscriber('/voice_command/state', Bool, self.state_callback)

    def initialize_listener(self):
        # kws_threshold is for keyword spotting (optional tuning)
        self.speech = LiveSpeech(
            verbose=False,
            sampling_rate=16000,
            buffer_size=2048,
            no_search=False,
            full_utt=False,
            lm=False,
            jsgf=self.rfm_path + JSGF_FILE,  # Point to grammar file
            dic=self.rfm_path + DICT_FILE   # Point to dictionary file
        )
 
    def state_callback(self, msg):
        # Update the state immediately when button is pressed
        self.is_recording = msg.data
        status = "UNMUTED" if self.is_recording else "MUTED"
        print(NODE_NAME + " Microphone is now " + status)

    def load_locations(self):
        with open(self.mrb_path + LOCATIONS_JSON_FILE) as json_file:
            locations_data = json.load(json_file)
            self.location_list = [re.sub(r'[^\w]', ' ', str(loc["name"]).lower()).strip() for loc in locations_data]

    def generate_dictionary_file(self):
        # 1. Prepare Environment
        master_map = self.load_master_dict()
        
        # 2. Process Input List
        all_inputs = self.location_list + basic_word_list
        word_list = self.clean_and_tokenize(all_inputs)
        
        # 3. Generate Output Lines
        output_lines = []
        missing_words = []
        
        for word in word_list:
            word_upper = word.upper() # Dictionary keys are usually UPPERCASE
            
            if word_upper in master_map:
                # Format: "word PHONEMES"
                # We save the key as lowercase for your .dic file preference, 
                # or keep it as is. Pocketsphinx accepts the word text on the left.
                entry = "%s\t%s" % (word.lower(), master_map[word_upper].upper())
                output_lines.append(entry)
            else:
                missing_words.append(word)

        # 4. Handle Missing Words
        if missing_words:
            error_string = "The following words were not found in the local dictionary: "
            for w in missing_words:
                error_string = error_string + w
            print(NODE_NAME + " ERROR: " + error_string)
            print(NODE_NAME + " You must add these manually or rename them.")

            # Send message to error topic to notify UI
            self.error_publisher.publish(error_string + ". Voice command cannot be activated.")

        # 5. Write Sorted File
        output_lines.sort() # ALPHABETICAL SORT IS CRITICAL
        
        with open(self.rfm_path + DICT_FILE, "w") as f:
            for line in output_lines:
                f.write(line + "\n")

        print(NODE_NAME + " Generated dictionary file '%s' with %d words." % (self.rfm_path + DICT_FILE, len(output_lines)))

    def generate_grammar_file(self):
        print(NODE_NAME + " Generating Grammar Rules...")

        # 1. Prepare Location List
        # We simply convert everything to its token form
        formatted_locations = set()
        for loc in self.location_list:
            formatted_locations.add(self.normalize_phrase(loc))
            
        # Sort alphabetically for clean file
        loc_rule_str = " | ".join(sorted(list(formatted_locations)))

        # 2. Write File
        with open(self.rfm_path + JSGF_FILE, "w") as f:
            f.write("#JSGF V1.0;\n\n")
            f.write("grammar robot_cmd;\n\n")
            
            f.write("/* --- MAIN PUBLIC RULE --- */\n")
            f.write("public <command> = <complex_command> | <simple_command>;\n\n")

            f.write("/* --- SIMPLE COMMANDS --- */\n")
            f.write("<simple_command> = %s;\n\n" % " | ".join(simple_commands))

            f.write("/* --- COMPLEX COMMANDS (Action + Location + Optional Priority) --- */\n")
            f.write("<complex_command> = (<move_action> | <handling_action> | <transport_action>) [ <priority_flag> ];\n\n")
            
            f.write("// --- ACTION RULES --- */\n")
            f.write("<move_action> = (%s) <location>;\n" % " | ".join(move_actions))
            f.write("<handling_action> = (%s);\n" % " | ".join(handling_actions))
            f.write("<transport_action> = (%s) <location> to <location>;\n\n" % " | ".join(transport_actions))
            
            f.write("// --- LOCATIONS --- */\n")
            f.write("// Machines and Places are mixed here as single tokens\n")
            f.write("<location> = %s;\n\n" % loc_rule_str)
            
            f.write("// --- PRIORITY --- */\n")
            f.write("<priority_flag> = priority (%s);\n" % " | ".join(priority_keywords))

        print(NODE_NAME + " Grammar file generated with %d locations." % len(formatted_locations))

    def run(self):
        # Load locations and generate dictionary (.dict) and gramma (.jgsf) files
        self.load_locations()
        self.generate_dictionary_file()
        self.generate_grammar_file()

        # Initialize pocketsphinx's LiveSpeech object based on established grammar and dictionary
        try:
            self.initialize_listener()
            # The infinite loop
            for phrase in self.speech:
                # Check if in recording mode
                if self.is_recording:
                    text = str(phrase)
                    print(NODE_NAME + " Recognized: " + text)
                    self.command_publisher.publish(text)
                else:
                    # If not recording, just wait
                    pass
                    
                if rospy.is_shutdown():
                    break
        except RuntimeError:
            rospy.logerr("LiveSpeech object could not be created. Possibly due to incomplete dictionary")

    #region ### HELPER FUNCTIONS FOR CREATING DICTIONARY AND GRAMMAR FILE ###
    def clean_phonemes(self, phoneme_str):
        """
        Input: "AH0 B AE1 N D AH0 N"
        Output: "AH B AE N D AH N"
        Removes the numbers 0, 1, 2 from the sounds.
        """
        # Remove digits using Regex
        return re.sub(r'\d+', '', str(phoneme_str))

    def load_master_dict(self):
        phoneme_map = {}
        with open(self.rfm_path + MASTER_DICT, "r") as f:
            for line in f:
                if line.startswith(";;;"): continue
                # Split by 2 spaces (standard CMU format)
                # Sometimes it's tab separated, let's be safe and split by whitespace
                parts = line.strip().split("  ") 
                if len(parts) < 2:
                    # Try tab split just in case
                    parts = line.strip().split("\t")
                
                if len(parts) >= 2:
                    word = parts[0]
                    raw_phonemes = " ".join(parts[1:])
                    # STRIP THE NUMBERS HERE
                    phoneme_map[word] = self.clean_phonemes(raw_phonemes.upper())
        return phoneme_map

    def clean_and_tokenize(self, phrases):
        """
        1. Converts 'Machine 5' -> 'machine five'
        2. Splits into unique words ['machine', 'five']
        """
        unique_words = set()
        
        for phrase in phrases:
            # Convert to lower case
            clean_phrase = phrase.lower()
            
            # Split into words to check for numbers
            temp_words = []
            for word in clean_phrase.split():
                if word.isdigit():
                    # Convert "1" -> "one"
                    word_as_text = n2w(int(word))
                    # Remove dashes ("twenty-one" -> "twenty one") for dictionary safety
                    word_as_text = word_as_text.replace("-", " ")
                    temp_words.append(word_as_text)
                else:
                    temp_words.append(word)
            
            # Flatten the list and add to set
            final_str = " ".join(temp_words)
            for w in final_str.split():
                unique_words.add(w)
                
        return sorted(list(unique_words)) # Return sorted list
    
    def normalize_phrase(self, phrase):
        """
        'Machine 1' -> 'machine-one'
        """
        clean_parts = []
        for part in phrase.lower().split():
            if part.isdigit():
                word_num = n2w(int(part)).replace(" ", "-").lower()
                clean_parts.append(word_num)
            else:
                clean_parts.append(part)
        return " ".join(clean_parts)
    #endregion
    
if __name__ == '__main__':
    try:
        node = VoiceCommandNode()
        node.run()
    except rospy.ROSInterruptException:
        pass