import speech_recognition as sr 
import spacy
from spacy.matcher import Matcher 
from spacy.lang.en import English
from spacy.matcher import PhraseMatcher
sample_rate = 48000
#Chunk is like a buffer. It stores 2048 samples (bytes of data) 
#here. 
#it is advisable to use powers of 2 such as 1024 or 2048 
chunk_size = 2048
#Initialize the recognizer 
r = sr.Recognizer() 
 

with sr.Microphone(sample_rate = sample_rate, 
						chunk_size = chunk_size) as source: 
	#wait for a second to let the recognizer adjust the 
	#energy threshold based on the surrounding noise level 
	r.adjust_for_ambient_noise(source) 
	print ("Say Something")
	#listens for the user's input 
	audio = r.listen(source)

	try: 
		print("Listening Done")
		# out = str(r.recognize_google(audio))
		out = r.recognize_google(audio)

		print ("you said: " + out )
	
	#error occurs when google could not understand what was said 
	
	except sr.UnknownValueError: 
		print("Google Speech Recognition could not understand audio") 
	
	except sr.RequestError as e: 
		print("Could not request results from Google \
								Speech Recognition service; {0}".format(e)) 

nlp = spacy.load("en_core_web_sm")
# doc = nlp("Turn right at the end of lab B. Turn right at the end of hallway. Washroom is on your Right..  ")
doc = nlp(out)

# print(doc[0:10].text)

# General keyword
wordmatcher = PhraseMatcher(nlp.vocab, attr="LOWER")
# patterns = [nlp.make_doc(unicode(name,"utf-8")) for name in ["left", "right","second right","third","last","keep","stay","till","until","not","available"]]
patterns = [nlp.make_doc(name) for name in ["left", "right","second","third","last","keep","stay"]]

# patterns =[u"left", u"right",u"second",u"third",u"last",u"keep",u"stay"]
wordmatcher.add("Names", None, *patterns)

# Neglect on your right condition
matcher = Matcher(nlp.vocab)
neg1 = [ {"POS": "ADP"},{"POS": "DET"},{"LOWER": u"right"}]
matcher.add("Vi_d", None,neg1)
matches = matcher(doc)
for match_id, start, end in matches:
    print("Neglect:", doc[start:end].text)
    pause=start
    print("start:", start)

## Dead End conditions
dead1= [{"LOWER": u"end"},{"POS": "ADP"},{"POS": "DET"},{"POS": "NOUN"}]
dead2=[{"POS": "DET"},{"LOWER": u"end"}]

# Add the pattern to the matcher and apply the matcher to the doc
matcher.add("Vi_d", None,dead1)
matcher.add("Vi_d", None,dead2)
endcond=False
for match_id, start, end in matcher(doc):
        print("Dead end keyword found")
        endcond=True
        print(doc[start:end].text)
        startc = start
        endc = end




#
pause=len(out)
s=u""
for match_id, start, end in wordmatcher(doc):
    if(start<pause):
        if(endcond):
            if((startc-4) < start or end > endcond):
                s = s +u"dead"
        s=s+" " +doc[start:end].text
        

print("Decoded sequence is", s)
    


