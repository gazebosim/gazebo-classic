

This is a simple nodejs server that receives POST commands over HTTPS.

The server supports username/password authentication (basic auth). The following users are hardcoded:
   username: 'myuser', password: 'mypass'
   username: 'joe', password: 'bloe'


These are the routes provided by the server:

   '/events/new': (HTTP method: POST) The server receives JSON messages. The user must be authenticated to
                  have access.
   '/':           (HTTP method: GET) The server serves a web page that lists all the received data on '/events/new'.
                  No authentication necessary.      
   '/login':      (HTTP method: GET) If authenticated correctly, the server returns the name of the user. This is
                  used to verify user credentials.

Installation:

  make sure nodejs is installed (skip if you have node installed already):
     sudo apt-get install nodejs nodejs-legacy npm
  
  now setup the local test server:
     cd <gazebo-src>/test/plugins/rest-web
     npm install

Running the server:
  
  To start the web server: npm start
  point your browser to (NOTE: use https, NOT http):

    https://localhost:3000


Send messages from Gazebo to the web server:

  start the simulation server (<gazebo-src>/test/plugins/rest-web/gzserver.bash):

    gzserver --verbose  -s libRestWebPlugin.so ../../worlds/rest_web.world
    

  start the simulation client (<gazebo-src>/test/plugins/rest-web/gzserver.bash):

    gzclient --verbose -g libRestUiPlugin.so menu="REST-web" title="test server" label="Login" url="https://localhost:3000"

  You should run gzserver.bash and gzclient.bash from their directory, otherwise you will get an error (Could not open file[../../worlds/rest_web.world]).
  In gzclient, there should be a new menu titled: "REST-web". Log in with user 'myuser' and password 'mypass'. Then, start and stop the simulation, you should see events published to the web site.



