# -*- coding: utf-8 -*-

# This sample demonstrates handling intents from an Alexa skill using the Alexa Skills Kit SDK for Python.
# Please visit https://alexa.design/cookbook for additional examples on implementing slots, dialog management,
# session persistence, api calls, and more.
# This sample is built using the handler classes approach in skill builder.
import logging #used to record key actions or errors
import ask_sdk_core.utils as ask_utils #check if request is of launch or intent type

from ask_sdk_core.skill_builder import SkillBuilder #used to configure and build alexa skill
from ask_sdk_core.dispatch_components import AbstractRequestHandler #used as a base class for creating custom request handlers in an Alexa skill. This class is essential for defining how specific Alexa intents and requests are handled. The 2 main methods it uses are 1) can_handle 2) handle) with 2 inputs in each of the methods.
from ask_sdk_core.dispatch_components import AbstractExceptionHandler #used as a base class for handling errors and exceptions in Alexa skills.The 2 main methods it uses are 1) can_handle 2) handle) each method has 3 inputs.
from ask_sdk_core.handler_input import HandlerInput #encapsulates all information related to an incoming request within an Alexa skill. It provides access to both the request details and tools to create a response.
#from labels import label_to_id
import requests # used to send HTTP requests to an external api/server
from ask_sdk_model import Response #response structure Alexa will return to the user after processing a request.
#import random 
#setup logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

ngrok = 'https://31f6-2607-fb91-799b-d66a-6823-d882-98e-419d.ngrok-free.app' #assigns an ngrok URL to the variable ngrok. This URL allows Alexa skills (frontend) API to communicate with a server running on Jetson Nano by exposing it to the internet.

# for each intent, there is a need to create a function which determines 
# the type of process needs to be done for the intent
# in this case, for each intent, a specific endpoint is called
# using ngrok url which then send request to the server (jetson flask app)
# and receieve the response. Then, the function just return the message 
# content of the response from the server


class LaunchRequestHandler(AbstractRequestHandler):
    """Handler for Skill Launch."""
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool

        return ask_utils.is_request_type("LaunchRequest")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speak_output = "Hello! Welcome to Object Selection"

        return (
            handler_input.response_builder #provides methods to create Alexa responses with speech, reprompts, etc
                .speak(speak_output)
                .ask(speak_output)
                .response
        )

class GetLabelHandler(AbstractRequestHandler):
    """Handler for Getting label."""
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool

        return (ask_utils.is_request_type("IntentRequest")(handler_input) and 
            ask_utils.is_intent_name("GetObjectLabel")(handler_input))

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response

        name = ask_utils.request_util.get_slot(handler_input, "name").value # retrieve slot values from the request

        res = requests.get(f'{ngrok}/id?name={name}') #fetch data from ngrok server. In this case specifically retrieves label based on object name.
        if res.status_code == 200: #success
            speak_output = res.json()['message']
        else:
            speak_output = 'invalid request'

        return (
            handler_input.response_builder
                .speak(speak_output)
                .ask(speak_output)
                .response
        )
        
class PickupHandler(AbstractRequestHandler):
    """Handler for pickup."""
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool

        return (ask_utils.is_request_type("IntentRequest")(handler_input) and 
            ask_utils.is_intent_name("PickupObject")(handler_input))

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        name = ask_utils.request_util.get_slot(handler_input, "name").value
        res = requests.get(f'{ngrok}/pickup?name={name}')
        if res.status_code == 200:
            speak_output = res.json()['message']
        else:
            speak_output = 'invalid request'

        return (
            handler_input.response_builder
                .speak(speak_output)
                .ask(speak_output)
                .response
        )

# class to handle GetObjectLocation intent
# class name (e.g. GetLocationHandler) does not matter but it should be similar to intent name
class GetLocationHandler(AbstractRequestHandler):
    """Handler for Getting location."""
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        
        # this line tells alexa that if GetObjectLocation intent is detected, call the handle function inside this class
        return (ask_utils.is_request_type("IntentRequest")(handler_input) and 
            ask_utils.is_intent_name("GetObjectLocation")(handler_input))

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        # first, it gets the input argument of IntentRequest. Each intent has a set of input arguments, 
        # in this case, the intent has one argument called name which refers to the object name
        name = ask_utils.request_util.get_slot(handler_input, "name").value
        
                
        # Alexa Frontend then sends a request to the server (jetson flask app) with specific endpoint and specific argument
        # in this case, the endpoint is location (this is defined in the flask app on the jetson) and the argument is name
        res = requests.get(f'{ngrok}/location?name={name}')
        # check if the server responded to the request (200 means successful)
        if res.status_code == 200:
            # Frontned extracts the message part of the response and speaks it
            speak_output = res.json()['message']
        else:
            speak_output = 'invalid request'
        
        return (
            handler_input.response_builder 
                .speak(speak_output)
                .ask(speak_output)
                .response
        )


class GetAvailableHandler(AbstractRequestHandler):
    """Handler for Getting object availability."""
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool

        return (ask_utils.is_request_type("IntentRequest")(handler_input) and 
            ask_utils.is_intent_name("IsObjectAvailable")(handler_input))

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        name = ask_utils.request_util.get_slot(handler_input, "name").value

        # following request takes longer than 8 seconds
        res = requests.get(f'{ngrok}/available?name={name}')
        if res.status_code == 200:
            speak_output = res.json()['message']
        else:
            speak_output = 'invalid request'

        return (
            handler_input.response_builder
                .speak(speak_output)
                .ask(speak_output)
                .response
        )
        
        
class GetObjectsHandler(AbstractRequestHandler):
    """Handler for Getting objects currently present in the workspace."""
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool

        return (ask_utils.is_request_type("IntentRequest")(handler_input) and 
            ask_utils.is_intent_name("GetWorkspaceObjects")(handler_input)) #only call this specific class when intent is GetWorkspaceObjects

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        res = requests.get(f'{ngrok}/objects')
        if res.status_code == 200:
            speak_output = res.json()['message']
        else:
            speak_output = 'invalid request'

        return (
            handler_input.response_builder
                .speak(speak_output)
                .ask(speak_output)
                .response
        )

class HelloWorldIntentHandler(AbstractRequestHandler):
    """Handler for Hello World Intent."""
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return ask_utils.is_intent_name("HelloWorldIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speak_output = "Hello World!"

        return (
            handler_input.response_builder
                .speak(speak_output)
                # .ask("add a reprompt if you want to keep the session open for the user to respond")
                .response
        )


class HelpIntentHandler(AbstractRequestHandler):
    """Handler for Help Intent."""
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return ask_utils.is_intent_name("AMAZON.HelpIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speak_output = "You can say hello to me! How can I help?"

        return (
            handler_input.response_builder
                .speak(speak_output)
                .ask(speak_output)
                .response
        )


class CancelOrStopIntentHandler(AbstractRequestHandler):
    """Single handler for Cancel and Stop Intent."""
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return (ask_utils.is_intent_name("AMAZON.CancelIntent")(handler_input) or
                ask_utils.is_intent_name("AMAZON.StopIntent")(handler_input))

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speak_output = "Goodbye!"

        return (
            handler_input.response_builder
                .speak(speak_output)
                .response
        )

class FallbackIntentHandler(AbstractRequestHandler):
    """Single handler for Fallback Intent."""
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return ask_utils.is_intent_name("AMAZON.FallbackIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        logger.info("In FallbackIntentHandler")
        speech = "Hmm, I'm not sure. You can say Hello or Help. What would you like to do?"
        reprompt = "I didn't catch that. What can I help you with?"

        return handler_input.response_builder.speak(speech).ask(reprompt).response

class SessionEndedRequestHandler(AbstractRequestHandler):
    """Handler for Session End."""
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return ask_utils.is_request_type("SessionEndedRequest")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response

        # Any cleanup logic goes here.

        return handler_input.response_builder.response


class IntentReflectorHandler(AbstractRequestHandler):
    """The intent reflector is used for interaction model testing and debugging.
    It will simply repeat the intent the user said. You can create custom handlers
    for your intents by defining them above, then also adding them to the request
    handler chain below.
    """
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return ask_utils.is_request_type("IntentRequest")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        intent_name = ask_utils.get_intent_name(handler_input)
        speak_output = "You just triggered " + intent_name + "."

        return (
            handler_input.response_builder
                .speak(speak_output)
                # .ask("add a reprompt if you want to keep the session open for the user to respond")
                .response
        )


class CatchAllExceptionHandler(AbstractExceptionHandler):
    """Generic error handling to capture any syntax or routing errors. If you receive an error
    stating the request handler chain is not found, you have not implemented a handler for
    the intent being invoked or included it in the skill builder below.
    """
    def can_handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> bool
        return True

    def handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> Response
        logger.error(exception, exc_info=True)

        speak_output = "Sorry, I had trouble doing what you asked. Please try again."

        return (
            handler_input.response_builder
                .speak(speak_output)
                .ask(speak_output)
                .response
        )

# The SkillBuilder object acts as the entry point for your skill, routing all request and response
# payloads to the handlers above. Make sure any new handlers or interceptors you've
# defined are included below. The order matters - they're processed top to bottom.



sb = SkillBuilder() # register request and exception handlers

#add request handlers

sb.add_request_handler(LaunchRequestHandler())
sb.add_request_handler(GetLabelHandler())
sb.add_request_handler(GetLocationHandler())
sb.add_request_handler(GetAvailableHandler())
sb.add_request_handler(PickupHandler())
sb.add_request_handler(GetObjectsHandler())
sb.add_request_handler(HelloWorldIntentHandler())
sb.add_request_handler(HelpIntentHandler())
sb.add_request_handler(CancelOrStopIntentHandler())
sb.add_request_handler(FallbackIntentHandler())
sb.add_request_handler(SessionEndedRequestHandler())
sb.add_request_handler(IntentReflectorHandler()) # Ensure that IntentReflectorHandler is last so it doesn't override custom intent handlers

#add exception handlers
sb.add_exception_handler(CatchAllExceptionHandler())

lambda_handler = sb.lambda_handler() # routes incoming Alexa requests to the appropriate handler based on the type of request.