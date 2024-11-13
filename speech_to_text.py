import speech_recognition as sr
import time
# from speech_recognition.recognizers import google, whisper

def command(command):
    """Processes the command given
    Args:
        command: string: the voice command from the user
    
    Returns:
        (value, processed_command)
    """
    parsed_command = command.split()
    
    # Process options 1 through 4:
    if len(parsed_command) != 1:
        # Remove value from the command and store in a different variable, then merge the command back together
        
        value = parsed_command.pop(1)
        processed_command = " ".join(parsed_command)

        # Check if value is a valid float if the command is valid
        try:
            value = float(value)
            
        except:
            print("Value given must be a floating positive value as per possible commands. Try again.")
            return None, processed_command
        
        return value, processed_command
    
    else:
        return None, command


def voice():

    possible_commands = ("forward miles per hour", "forward mph", "backwards miles per hour", "backwards mph", "left degrees", "right degrees", "stop")
    print('Available Commands:')
    print(' 1. "Forward <x> miles per hour"')
    print(' 2. "Backwards <x> miles per hour"')
    print(' 3. "Left <x> degrees"')
    print(' 4. "Right <x> degrees"')
    print(' 5. "Stop"')

    print("Say command:")

    recog = sr.Recognizer()
    
    # Collect audio from live microphone
    with sr.Microphone() as source:
        audio = recog.listen(source)

    # Google Speech Recognition
    try:
        input = recog.recognize_google(audio)
        print(f"You said: {input}")

        # print("Command length:", type(input))
        value, processed_command = command(input)
        


        # Check if command is in the valid set of commands, if not return an error
        if processed_command not in possible_commands:
            print("Not a valid command. Please follow format of available commands where <x> is a positive float")
            return
        else:

            # Find command
            if value and possible_commands != possible_commands[6]:
                sent_command = possible_commands.index(processed_command) + 1
                output = (value, sent_command)
            
            # Conditional for "stop"
            elif not value and possible_commands == possible_commands[6]:
                sent_command = 5
                output = (value, sent_command)
            # Conditional for a movement command with wrong value
            else:
                return 
            
    except sr.UnknownValueError:
        print("Could not understand audio! Please try again.")  
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))
    

    return output


print(voice())