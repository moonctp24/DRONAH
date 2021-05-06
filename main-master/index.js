//'use strict';
process.env.DEBUG = 'actions-on-google:*';
//const {actionssdk} = require('actions-on-google');

const {
    dialogflow,
    SignIn
} = require('actions-on-google');

//const app = dialogflow();
const app = dialogflow({
    // REPLACE THE PLACEHOLDER WITH THE CLIENT_ID OF YOUR ACTIONS PROJECT
    clientId: "912086039333-k90niva1c2qs81ru56qqr8kimmhu6rjv.apps.googleusercontent.com",
});

const functions = require('firebase-functions');
const admin = require('firebase-admin');

admin.initializeApp({
  credential: admin.credential.applicationDefault(),
  databaseURL: "https://dronah-955df.firebaseio.com/"
});

const auth = admin.auth();
const db = admin.database();
const WORK_INTENT = 'movement';



app.intent('Default Welcome Intent', conv => {
    conv.ask('Welcome');
});

app.intent(WORK_INTENT, (conv, {
  	start,
    control,
  	number,
    recognize,
    tracking
    
  
}) => {

    play(conv, start, control, number, recognize, tracking);
});


exports.dialogflowFirebaseFulfillment = functions.https.onRequest(app);

var target = 0;

function play(conv, start, con, val, rec, tra) {
  	console.log("start", start);
    console.log("control is  ", con);
    console.log(" value  ", val);
  	console.log("recognize", rec);
  	console.log("tracking", tra);
    return new Promise(function(resolve, reject) {
        devices_db = admin.database().ref('/DronAh');
        if (val) {
            target = val; //setting tracking number        
        }
        devices_db.child('value').set(target);      
		devices_db.child('start').set(start);        
   		devices_db.child('control').set(con);        
        if (rec){
        	devices_db.child('recognize').set(rec);        
        }
        devices_db.child('tracking').set(tra);	//tracking recognize 설정해야함 - and일때만 되게    
        return resolve;
    });
  
}