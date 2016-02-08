var fs = require('fs');
var https = require('https');
var privateKey  = fs.readFileSync('key.pem', 'utf8');
var certificate = fs.readFileSync('key-cert.pem', 'utf8');
var passport = require('passport');
var BasicStrategy = require('passport-http').BasicStrategy;
var bodyParser = require('body-parser');
var credentials = {key: privateKey, cert: certificate};
var express = require('express');
var util = require('util');

var app = express();

// your express configuration here
// var httpServer = http.createServer(app);
var httpsServer = https.createServer(credentials, app);

app.use(passport.initialize());
// parse application/json
app.use(bodyParser.json())

var users = [
    { id: 1, username: 'myuser', password: 'mypass', email: 'bob@example.com' }
  , { id: 2, username: 'joe', password: 'bloe', email: 'joe@example.com' }
];

function findByUsername(username, fn) {
  for (var i = 0, len = users.length; i < len; i++) {
    var user = users[i];
    if (user.username === username) {
      return fn(null, user);
    }
  }
  return fn(null, null);
}

// Use the BasicStrategy within Passport.
//   Strategies in Passport require a `verify` function, which accept
//   credentials (in this case, a username and password), and invoke a callback
//   with a user object.
passport.use(new BasicStrategy({
  },
  function(username, password, done) {
    // asynchronous verification, for effect...
    process.nextTick(function () {

      // Find the user by username.  If there is no user with the given
      // username, or the password is not correct, set the user to `false` to
      // indicate failure.  Otherwise, return the authenticated `user`.
      findByUsername(username, function(err, user) {
        if (err) { return done(err); }
        if (!user) { return done(null, false); }
        if (user.password != password) { return done(null, false); }
        return done(null, user);
      })
    });
  }
));

all_events = [];

// curl -v -I http://127.0.0.1:3000/
// curl -v -I --user bob:secret http://127.0.0.1:3000/
app.get('/login',
   passport.authenticate('basic', { session: false }),
   function(req, res){
     console.log("-----\nLogin:" + util.inspect(req.user));
     all_events.push(req.user);
     res.jsonp( req.user );
     console.log('LOGIN DONE');
  });


app.get('/',
  // Authenticate using HTTP Basic credentials, with session support disabled.
  // passport.authenticate('basic', { session: false }),
  function(req, res){
    //res.jsonp({ username: req.user.username, email: req.user.email });
    console.log('serving all events');

    res.write('<html>');
    // refresh the page every sec
    res.write("<meta http-equiv=\"refresh\" content=\"2\" >");
    res.write('<script>\n');
    res.write('events = ' + JSON.stringify(all_events) + ';\n' );
    res.write('</script>\n');

    res.write('<script src="script.js"></script>\n');

    res.write('<body onload="fillEventList()">');
    res.write(' <h1>' + all_events.length + ' events</h1>');
    colors = ['LightGrey', 'LightGreen'];
    for (var i = all_events.length-1; i >=0;  i--)
    {
      // get a nice formatted string for display
      var s = JSON.stringify(all_events[i], null, "  ");
      res.write('<div>' + i + '</div><pre style="background:' + colors[i%2] + '">' + s + '</pre>');
    }
    res.write(' <div id="list"/>');
    res.write('</body>');
    res.end('</html>');
  });

app.post('/events/new',
  passport.authenticate('basic', { session: false }),
  function(req, res){
   console.log("-----\nNEW EVENT:\n\n" + util.inspect(req.body));
   var r = { username: req.user.username, Event: req.body };
   all_events.push(r);
   res.jsonp(r);
  });

app.use(express.static(__dirname));

console.log('Listening on port 5000');
//httpServer.listen(80);
//httpsServer.listen(443);
httpsServer.listen(5000);

