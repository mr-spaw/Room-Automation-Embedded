from pydub import AudioSegment
from pydub.playback import play
audio=AudioSegment.from_file("/home/ben/Embedded/travis.mp3")
play(audio)
