1) Main code: racecar_controls --> voiceControl.py

   Ensure that the entire repository is open to access the right file paths to open dependents within voiceControl.py
   
3) Documentation: https://docs.google.com/document/d/1-2BnD_YAyNnpGMFKRgEGNweAAn-IqZWxb_EUYkL06Tc/edit?usp=sharing
   
4) To install Pybullet you must first have C++ on your computer
   - Windows:
        - pip install pybullet 
          or
          conda install conda-forge::pybullet

   - Linux:
        - sudo pip3 install by bullet 

5) To install SpeechRecognition and its required dependencies:
    - Windows (non-x84):
        - Install SpeechRecognition package:
            *pip install SpeechRecognition*
        - Install PyAudio package:
            *pip install PyAudio*
        - Install FLAC Encoder:
            
            i. Go to https://ftp.osuosl.org/pub/xiph/releases/flac/ then download
            the latest .zip folder

            ii. Extract that folder, then navigate into the extracted folder into either the Win32
            or Win64 folder depending on your Windows type. 

            iii. Double click *flac.exe* and *metaflac.exe*
    
    - Windows (x84):
        - Install SpeechRecognition package:
            *pip install SpeechRecognition*
        - Install PyAudio package:
            *pip install PyAudio*
    
    - Linux:
       - Install SpeechRecognition package:
           *pip install SpeechRecognition*
       - Install PyAudio package:
           *sudo apt-get install python-pyaudio python3-pyaudio*
          - Alternatively:
            *sudo apt-get install portaudio19-dev python-all-dev python3-all-dev && sudo pip install SpeechRecognition[audio]*

      
3) PyBullet Official Documentation: https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit?tab=t.0
