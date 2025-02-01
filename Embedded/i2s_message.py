from pydub import AudioSegment
from pydub.playback import play

def play_audio(auth):
    if auth == True:
        # Play the audio for successful authentication
        audio_authentication_successful = AudioSegment.from_file("/home/krs/Documents/Embedded/auth.m4a")
        play(audio_authentication_successful)
    else:
        # Play the audio for failed authentication
        audio_authentication_failed = AudioSegment.from_file("/home/krs/Documents/Embedded/deauth.m4a")
        play(audio_authentication_failed)


