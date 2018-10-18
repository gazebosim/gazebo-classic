from flask import Flask, request, make_response, render_template, abort
from flask import jsonify
# from Security import requires_auth
import logging
from logging.handlers import RotatingFileHandler

from lconfig import config
from lconfig import APP_LOGGER_NAME

#from services.Events.Events import eventService
#from services.Graphs.Graphs import graphService

#from services.Events.Tasks.tasks import process_messages_request, process_login_request

from werkzeug.exceptions import HTTPException

TAIL_MESSAGE = "-1"

app = Flask(__name__)
app.debug = True


#app.register_blueprint(eventService, url_prefix='/events')
#app.register_blueprint(graphService, url_prefix='/graphs')

LEARNING_COMPANION_VERSION = "0.0.20"


@app.route('/')
def index():
    """Home page handler"""
    json_response = ''
    try:
        json_response = {"Status": "Success"}
    except (HTTPException, Exception) as e:
        error_message = "Exception occurred.  Exception Message=" + e.message
        app.logger.error(error_message)
        json_response = {"Status": "Error", "Message": error_message}
    finally:
        return jsonify(json_response)


@app.route('/login')
def login():
    response = ''
    try:
        app.logger.debug("request.authorization={}".format(request.authorization))
        response = {"Message": "Learning Companion version: " + LEARNING_COMPANION_VERSION }
    except (HTTPException, Exception) as e:
        error_message = "Exception occurred. Exception Message=" + e.message
        app.logger.error(error_message)
        response = error_message
    finally:
        return jsonify(response)


@app.route('/events/new', methods=['POST', 'GET'])
def new_event():
	response = {"Status": "Success", "Event": "event_data_here"}
	try:
		event_data = request.get_json(force=False, silent=True)
		app.logger.debug("new event {}".format(event_data));
		
	except (HTTPException, Exception) as e:
		error_message = "Exception occurred. Exception Message=" + e.message
		app.logger.error(error_message)
		response = error_message
	finally:
		return jsonify(response)
		

def setup_logging():
    handler = RotatingFileHandler(config['logfile'], maxBytes=10000, backupCount=1)
    ch = logging.StreamHandler()

    handler.setLevel(logging.DEBUG)
    ch.setLevel(logging.DEBUG)

    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    ch.setFormatter(formatter)

    app.logger.addHandler(handler)

    logger = logging.getLogger(APP_LOGGER_NAME)
    logger.setLevel(logging.DEBUG)
    logger.addHandler(ch)
    logger.addHandler(handler)


if __name__ == '__main__':
    setup_logging()
    # context = ('/var/tmp/ssl.cert', '/var/tmp/ssl.key')
    # app.run(host='0.0.0.0', debug=True, ssl_context=context)
    app.run(host='0.0.0.0', debug=True)
    pass

else:
    setup_logging()
