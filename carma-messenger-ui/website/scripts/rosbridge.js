/*
 * Copyright (C) 2018-2020 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

/***
 This file shall contain ROS related function calls.
****/

// *** Global variables ***
var sound_counter = 0;
var sound_counter_max = 3; //max # of times the sounds will be repeated.
var sound_played_once = false;
var isModalPopupShowing = false;

// Deployment variables
var ip = CarmaJS.Config.getIP();


// Topics
var driver_discovery = '/driver_discovery';

// ROS related
var ros = new ROSLIB.Ros();

//Elements frequently accessed.
var audioElements = document.getElementsByTagName('audio');

/**
* Custom sleep used in enabling guidance
*/
function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}

/*
    Connection to ROS
*/
function connectToROS() {

    try {
        // If there is an error on the backend, an 'error' emit will be emitted.
        ros.on('error', function (error) {
            document.getElementById('divLog').innerHTML += '<br/> ROS Connection Error.';
            console.log(error);
            document.getElementById('connecting').style.display = 'none';
            document.getElementById('connected').style.display = 'none';
            document.getElementById('closed').style.display = 'none';
            document.getElementById('error').style.display = 'inline';

        });

        // Find out exactly when we made a connection.
        ros.on('connection', function () {
            document.getElementById('divLog').innerHTML += '<br/> ROS Connection Made.';
            document.getElementById('connecting').style.display = 'none';
            document.getElementById('error').style.display = 'none';
            document.getElementById('closed').style.display = 'none';
            document.getElementById('connected').style.display = 'inline';

            //load widget
            CarmaJS.WidgetFramework.closeWidgets();
            CarmaJS.WidgetFramework.loadWidgets();
        });

        ros.on('close', function () {
            document.getElementById('divLog').innerHTML += '<br/> ROS Connection Closed.';
            document.getElementById('connecting').style.display = 'none';
            document.getElementById('connected').style.display = 'none';
            document.getElementById('closed').style.display = 'inline';

            //Show modal popup for when ROS connection has been abruptly closed.
            var messageTypeFullDescription = 'ROS Connection Closed.';
            messageTypeFullDescription += '<br/><br/>PLEASE CHECK YOUR ROS CONNECTION.';
            showModal(true, messageTypeFullDescription, false);
        });

        // Create a connection to the rosbridge WebSocket server.
        ros.connect('ws://' + ip + ':9090');

    }
    catch (err) {
        console.log(err);
    }
}

/*
    Get the CARMA version using the PHP script calling the docker container inspect to get the carma-config image version used.
*/
function getCARMAVersion() {
    var request = new XMLHttpRequest();
    var url = "scripts/carmaVersion.php";
    request.open("POST", url, true);
    request.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");

    request.onreadystatechange = function () {
        if (request.readyState == 4 && request.status == 200) {
            showCARMAVersion(request.responseText);
        }
    }
    request.send();
}
/*
    Show the CARMA version under the footer tag.
*/
function showCARMAVersion(response) {
    var elemSystemVersion = document.getElementsByClassName('systemversion');
    console.log(response);
    if(response!=null && response.trim().length>0)
        elemSystemVersion[0].innerHTML = response;
    else 
        elemSystemVersion[0].innerHTML = '--';
}

/*
    Onload function that gets called when first loading the page and on page refresh.
*/
window.onload = function () {
    //Check if localStorage/sessionStorage is available.
    if (typeof (Storage) !== 'undefined') {

        if (!SVG.supported) {
            console.log('SVG not supported. Some images will not be displayed.');
        }

        // Adding Copyright based on current year
        var elemCopyright = document.getElementsByClassName('copyright');
        elemCopyright[0].innerHTML = '&copy LEIDOS ' + new Date().getFullYear();

        // Adding CARMA Version based on the carma-config docker container version
        getCARMAVersion();

        //Refresh requires connection to ROS.
        connectToROS();

    } else {
        // Sorry! No Web Storage support..
        console.log('Sorry, cannot proceed unless your browser support HTML Web Storage Objects. Please contact your system administrator.');

    }
}

