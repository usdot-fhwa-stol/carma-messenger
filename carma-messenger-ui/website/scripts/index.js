/*
 * Copyright (C) 2018-2021 LEIDOS.
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
var retry_counter=0;
var isROSConnected=false;

$(document).ready(function () {       
    $("#jqxLoader").jqxLoader({width: 150, height: 100, imagePosition: 'center', isModal: true});
    $('#btnStart').on('click', function () {

            //If launching backend platform, call the waitForROSConnection since starting backend service takes time;
            if($('input[name=launchplatform]').prop('checked')) {
                $('#jqxLoader').jqxLoader('open'); 
                waitForROSConnection();
            }              
            else{
                //check immediate ROS connection status
                connectToROS(true); 
            }
    });
    $('form').submit(function(event){
        console.log('submit form');
        console.log($('input[name=launchplatform]').prop('checked'));
        console.log($('input[name=rosbagrecorder]').prop('checked'));
        event.preventDefault();
        $.ajax({
            type:'POST',
            url:'scripts/launchPlatform.php',
            data: {
                'launchplatform':$('input[name=launchplatform]').prop('checked'),
                'rosbagrecorder':$('input[name=rosbagrecorder]').prop('checked')
            },
            dataType:'json'
        })
        .done(function(data){
            console.log('form done' + data)
        });
    });
});
function waitForROSConnection() {

    //  call a setTimeout when the loop is called
    setTimeout(function () {   
        //check ROS here
        connectToROS(false); 

        //increment the counter
        retry_counter++;       

        //if the counter < 4, call the loop function
        if (retry_counter < CarmaJS.Config.getRosConnectionRetry() && isROSConnected == false) {
            console.log('Awaiting ROS connection status ...attempt: ' + retry_counter);
            waitForROSConnection();             //  ..  again which will trigger another          
        }      
        else { //If over max tries
            if (retry_counter >= CarmaJS.Config.getRosConnectionRetry()){
                $('#jqxLoader').jqxLoader('close');  
                $('#messenger-connection-fail-alert').show();
                retry_counter = 0;
            }                
        }
    }, CarmaJS.Config.getRosConnectionWaitTime())//  ..  setTimeout()
}
function connectToROS(displayROSConnectionClosed) {
        try {
            // ROS related
            var ip = CarmaJS.Config.getIP();
            var ros = new ROSLIB.Ros();

            // If there is an error on the backend, an 'error' emit will be emitted.
            ros.on('error', function (error) {
                console.log(error);
            });

            // Find out exactly when we made a connection.
            ros.on('connection', function () {
                console.log('success');
                window.location.href = "../main.html";                        
            });

            ros.on('close', function () {
                console.log('close');
                if(displayROSConnectionClosed){
                    $('#messenger-connection-fail-alert').show(); 
                }
                    
            });

            // Create a connection to the rosbridge WebSocket server.
            ros.connect('ws://' + ip + ':9090');
        }
        catch (err) {
            console.log(err);
        }
}