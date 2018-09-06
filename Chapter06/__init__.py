from mycroft import MycroftSkill, intent_file_handler
import time


class RecieveKnock(MycroftSkill):
    def __init__(self):
        MycroftSkill.__init__(self)
        self.handle_who=False

    @intent_file_handler('knock.recieve.intent')
    def handle_knock_recieve(self, message):
        response =self.get_response('who.is.there')
        print ("knock knock",response)
        response2= response + " who?"
	#self.speak(response2)
        response3 =self.get_response(announcement=response2)
        self.speak_dialog('veryfunny')
        time.sleep(12)
        
    def stop(self):
        pass

def create_skill():
    return RecieveKnock()


