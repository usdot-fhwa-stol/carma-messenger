/***
* Create a unique namespace for each plugin widget to minimize collision of same name variables or functions.
***/
CarmaJS.registerNamespace("CarmaJS.WidgetFramework.eventManagement");

CarmaJS.WidgetFramework.eventManagement = (function () {

    //*** Private Variables ***
    //Listeners
    //*** Widget Install Folder ***
    //Currently the URL path from document or window are pointing to the page, not the actual folder location.
    //Therefore this needs to be hardcoded.
    var installfoldername = 'widgets/eventManagement/';
    var broadcasting_btn_status_t = {
        "STARTED": 0,
        "STOPPED": 1,
        "UNKNOWN": 2,
        "START": 3,
        "STOP": 3,
    };
    var BC_FQ_10HZ = "10HZ";
    var broadcasting_btn_status = broadcasting_btn_status_t.UNKNOWN;

    /***
       * Custom widgets using JQuery Widget Framework.
       * NOTE: that widget framework namespace can only be one level deep.
       * This should not collide with other widgets as loadCustomWidget will be calling private widgets.
       ***/
    var checkSessionVariables = function()
    {
        if(EventSessionFormFields !=null)
        {
            if( EventSessionFormFields !=null &&  EventSessionFormFields.BCStatus=="true")
            {
                $('#eventBroadcastBtn').val('Stop broadcasting');
                $('#eventBroadcastBtn').css('background-color','red');
                $('#BroadcastingFrequencySpan').text(BC_FQ_10HZ);
                $('#BroadcastingFrequencySpan').removeClass('hide');
                $('#BroadcastingFrequencyLabelTitle').removeClass('hide');
                $('#BroadcastingFrequencySpan').addClass('show');
                $('#BroadcastingFrequencyLabelTitle').addClass('show');
            }
            if( EventSessionFormFields !=null &&  EventSessionFormFields.upTrack !=null)
            {
                $('#UpTrack').val(EventSessionFormFields.upTrack);
                $('#UpTrack').prop('disabled',true);
            }
            if( EventSessionFormFields !=null &&  EventSessionFormFields.downTrack !=null)
            {
                $('#DownTrack').val(EventSessionFormFields.downTrack);
                $('#DownTrack').prop('disabled',true);
            }
            if( EventSessionFormFields !=null &&  EventSessionFormFields.minGap !=null)
            {
               $('#MinGap').val( EventSessionFormFields.minGap);
               $('#MinGap').prop('disabled',true);
            }
            if( EventSessionFormFields !=null &&  EventSessionFormFields.advisorySpeed !=null)
            {
               $('#AdvisorySpeed').val(EventSessionFormFields.advisorySpeed);
               $('#AdvisorySpeed').prop('disabled',true);
            }
        }
    }
    var changeFields = function(toDisable){
        var UpTrackValue =  $('#UpTrack');
        var DownTrackValue =  $('#DownTrack');
        var MinGapValue =  $('#MinGap');
        var AdvisorySpeedValue =  $('#AdvisorySpeed');

        if(toDisable == true)
        {
            UpTrackValue.prop('disabled', true);
            DownTrackValue.prop('disabled', true);
            MinGapValue.prop('disabled', true);
            AdvisorySpeedValue.prop('disabled', true);

        }else{
            UpTrackValue.prop('disabled', false);
            DownTrackValue.prop('disabled', false);
            MinGapValue.prop('disabled', false);
            AdvisorySpeedValue.prop('disabled', false);
        }
    }
    var changeBtn = function(btn,toStatus){
          if(toStatus.toLowerCase().includes('start'))
          {
            $(btn).css('background-color', '#7fb83d');
            $(btn).val("Start Broadcast");
            $(btn).addClass('btn-success');
            $(btn).removeClass('btn-danger');
          }
          else
          {
            $(btn).css('background-color', 'red');
            $(btn).val("Stop Broadcast");
            $(btn).removeClass('btn-success');
            $(btn).addClass('btn-danger');
          }
    }

    var send_start_broadcasting_request = function (btn){
        console.log("send start broadcasting request");
        var sendStartBCRequest = new ROSLIB.Service({
            ros: ros,
            name: '/start_broadcasting_traffic_event',
            serviceType: 'cav_srvs/SetTrafficEvent.h'
        });

        //event UI form
        var UpTrackValue =  $('#UpTrack').val();
        var DownTrackValue =  $('#DownTrack').val();
        var MinGapValue =  $('#MinGap').val();
        var AdvisorySpeedValue =  $('#AdvisorySpeed').val();
        if(UpTrackValue=="" || isNaN(UpTrackValue) 
            || UpTrackValue > CarmaJS.Config.getUpTrackRange().MAX 
            || UpTrackValue < CarmaJS.Config.getUpTrackRange().MIN)
        {
            alert("Up Track value is required, should be greater than "+CarmaJS.Config.getUpTrackRange().MIN+" and less than "+ CarmaJS.Config.getUpTrackRange().MAX);
            return false;
        }

        if(DownTrackValue=="" || isNaN(DownTrackValue) 
        || DownTrackValue > CarmaJS.Config.getDownTrackRange().MAX 
        || DownTrackValue < CarmaJS.Config.getDownTrackRange().MIN){
            alert("Down Track value  is required and should be greater than "+CarmaJS.Config.getDownTrackRange().MIN+" and less than "+ CarmaJS.Config.getDownTrackRange().MAX);
            return false;
        }

        if(MinGapValue==""|| isNaN(MinGapValue) 
        || MinGapValue > CarmaJS.Config.getMinGapRange().MAX 
        || MinGapValue < CarmaJS.Config.getMinGapRange().MIN){
            alert("Minimum Gap value  is required and should be greater than "+CarmaJS.Config.getMinGapRange().MIN+" and less than "+ CarmaJS.Config.getMinGapRange().MAX);
            return false;
        }

        if(AdvisorySpeedValue==""|| isNaN(AdvisorySpeedValue) 
        || AdvisorySpeedValue > CarmaJS.Config.getAdvisorySpeedRange().MAX 
        || AdvisorySpeedValue < CarmaJS.Config.getAdvisorySpeedRange().MIN){
            alert("Advisory Speed value  is required and should be greater than "+CarmaJS.Config.getAdvisorySpeedRange().MIN+" and less than "+ CarmaJS.Config.getAdvisorySpeedRange().MAX);
            return false;
        }

        var request = new ROSLIB.ServiceRequest({
            up_track: parseFloat(UpTrackValue),
            down_track: parseFloat(DownTrackValue),
            minimum_gap: parseFloat(MinGapValue),
            advisory_speed: parseFloat(AdvisorySpeedValue)
        });

        try{
            sendStartBCRequest.callService(request, function(result) {});

            //check whether /outgoing_mobility_operation topic is broadcasting MobilityOperation data
            subscribe_outgoing_mobility_operation();
        }catch(ex){
            console.error("send start broadcasting request error: " + ex.message);
        }
    }
     // Subscribe to  /gps_common_fix topic
     var subscribe_gps_common_fix = function()
    {
        console.log('subscribe_gps_common_fix called');
         var listenerGPS = new ROSLIB.Topic({
            ros: ros,
            name: '/position/gps_common_fix',
            messageType: 'gps_common/GPSFix'
        });

        listenerGPS.subscribe(function (message) {
            console.log("function listenerGPS: "+ JSON.stringify(message));
            //update broadcast btn value to "Stop Broadcast" since it is already broadcasting data
            if(message.latitude!=null && message.latitude!='undefined' && message.latitude!=null && message.latitude!='undefined')
            {
                 $("#LatitudeSpan").text(message.latitude);
                 $("#LongitudeSpan").text(message.longitude);
                 $("#GPSStatusSpan").text("Good");
                 $("#GPSStatusSpan").addClass("success-text");
            }
            //listenerGPS.unsubscribe();
        });
    }

    //Subscribe to  /outgoing_mobility_operation topic
    var subscribe_outgoing_mobility_operation = function()
    {
         var listenerMobilityOperation = new ROSLIB.Topic({
            ros: ros,
            name: '/outgoing_mobility_operation',
            messageType: 'cav_msgs/MobilityOperation'
        });

        listenerMobilityOperation.subscribe(function (message) {
            listenerMobilityOperation.unsubscribe();
            console.log("function subscribeToMobilityOperation "+ JSON.stringify(message.strategy_params));

            //update broadcast btn value to "Stop Broadcast" since it is already broadcasting data
            changeBtn($("#eventBroadcastBtn"), 'stop');
            changeFields(true);
            // broadcasting frequency
            $('#BroadcastingFrequencySpan').text(BC_FQ_10HZ);
            $('#BroadcastingFrequencySpan').addClass('show');
            $('#BroadcastingFrequencyLabelTitle').addClass('show');
            $('#BroadcastingFrequencySpan').removeClass('hide');
            $('#BroadcastingFrequencyLabelTitle').removeClass('hide');

            broadcasting_btn_status = broadcasting_btn_status_t.STARTED;
            console.log('broadcasting_btn_status: ' + broadcasting_btn_status);

            //set session variables
            EventSessionFormFields.BCStatus = 'true';
            console.log(message.strategy_params.split(','));
            var params_arr = message.strategy_params.split(',');
            params_arr.forEach((item)=>{
                var key = item.split(':')[0];
                var value = item.split(':')[1];
                if(key.toLowerCase() == 'downtrack'){
                    EventSessionFormFields.downTrack = value;
                }else if(key.toLowerCase() == 'uptrack'){
                    EventSessionFormFields.upTrack = value;
                }else if(key.toLowerCase() == 'min_gap'){
                    EventSessionFormFields.minGap = value;
                } else if (key.toLowerCase() == 'advisory_speed'){
                    EventSessionFormFields.advisorySpeed = value;
                }
                 console.log(EventSessionFormFields);
            });

            if (message.data != null && message.data != 'undefined') {
                console.log("Received MobilityOperation message on " + listenerMobilityOperation.name+ ": " + message.data);
            }
        });
    }

    var send_stop_broadcasting_request = function (btn){
        console.log("send stop broadcasting request");
        var sendStopBCRequest = new ROSLIB.Service({
            ros: ros,
            name: '/stop_broadcasting_traffic_event',
            serviceType: 'std_srvs/Trigger'
        });
        var request = new ROSLIB.ServiceRequest({});
        try{
            sendStopBCRequest.callService(request, function(result) {
                console.log("send stop broadcasting request . Result message is: "+result.success);
                if(!result.success){
                    console.log("result failed");
                }
                else
                {
                    console.log("result is "+ result.message);
                    console.log('update btn to start');
                    changeBtn($("#eventBroadcastBtn"),'start');
                    $('#BroadcastingFrequencySpan').addClass('hide');
                    $('#BroadcastingFrequencyLabelTitle').addClass('hide');
                    $('#BroadcastingFrequencySpan').removeClass('show');
                    $('#BroadcastingFrequencyLabelTitle').removeClass('show');
                    changeFields(false);

                    //clear session variables

                    sessionStorage.removeItem('advisorySpeed');
                    sessionStorage.removeItem('upTrack');
                    sessionStorage.removeItem('downTrack');
                    sessionStorage.removeItem('BCStatus');
                    sessionStorage.removeItem('minGap');
                }
            });
        }catch(ex){
            console.error("send stop broadcasting request error: " + ex.message);
        }
    }

    $.widget("CarmaJS.eventManagement", {
        _create: function () {

              var divTitle=document.createElement('div');
              divTitle.className="event-management-title-div";

              var newLabelTitle=document.createElement('label');
              newLabelTitle.innerHTML = "Event Management";
              newLabelTitle.className="event-management-title";

              divTitle.append(newLabelTitle);
              $(this.element).append(divTitle);


              var divRow = document.createElement('div');
              divRow.className = 'form-group row';

              var divContentColOne = document.createElement('div');
              divContentColOne.className = 'col-8 inline-block-div float-left';

              var divNoteColTwo = document.createElement('div');
              divNoteColTwo.className = 'col-4 inline-block-div float-left';

               /***
                 Top Col - 1 : content
               **/
              //Row -1
              //Up Track
              var newLabelUpTrack=document.createElement('label');
              newLabelUpTrack.innerHTML = "Up Track";
              newLabelUpTrack.id="UpTrackLabel";
              newLabelUpTrack.className="form-label col-3 text-align-right";

              var newInputUpTrack = document.createElement('input');
              newInputUpTrack.name = 'UpTrack';
              newInputUpTrack.className="form-control col-2";
              newInputUpTrack.id = 'UpTrack';
              newInputUpTrack.type = "number";

              var newLabelUpTrackUnit = document.createElement('label');
              newLabelUpTrackUnit.innerHTML = "(Meter)";
              newLabelUpTrackUnit.className="col-1";

              //Latitude
              var newLabelLatitudeTitle = document.createElement('label');
              newLabelLatitudeTitle.innerHTML = "Current Latitude:";
              newLabelLatitudeTitle.id="LatitudeLabelTitle";
              newLabelLatitudeTitle.className="form-label offset-1 text-align-left ";

              var newSpanLatitude = document.createElement('span');
              newSpanLatitude.innerHTML = "N/A";
              newSpanLatitude.id="LatitudeSpan";
              newSpanLatitude.className="form-label  text-align-left padding-left-10";

              var divUpTrack = document.createElement('div');
              divUpTrack.className = 'form-group row';

              divUpTrack.append(newLabelUpTrack);
              divUpTrack.append(newInputUpTrack);
              divUpTrack.append(newLabelUpTrackUnit);

              //Latitude element and Up Track UI element are at the same row.
              divUpTrack.append(newLabelLatitudeTitle);
              divUpTrack.append(newSpanLatitude);
              divContentColOne.append(divUpTrack);

              //Row 2
              //Down Track
              var newLabelDownTrack=document.createElement('label');
              newLabelDownTrack.innerHTML = "Down Track";
              newLabelDownTrack.id="DownTrackLabel";
              newLabelDownTrack.className="form-label col-3 text-align-right";

              var newInputDownTrack = document.createElement('input');
              newInputDownTrack.name = 'DownTrack';
              newInputDownTrack.id = 'DownTrack';
              newInputDownTrack.type  = "number";
              newInputDownTrack.className="form-control col-2";

              var newLabelDownTrackUnit=document.createElement('label');
              newLabelDownTrackUnit.innerHTML = "(Meter)";
              newLabelDownTrackUnit.className="col-1";

              var divDownTrack = document.createElement('div');
              divDownTrack.className = 'form-group row';

              //Longitude
              var newLabelLongitudeTitle = document.createElement('label');
              newLabelLongitudeTitle.innerHTML = "Current Longitude:";
              newLabelLongitudeTitle.id="LatitudeLabelTitle";
              newLabelLongitudeTitle.className="form-label offset-1 text-align-left ";

              var newSpanLongitude = document.createElement('span');
              newSpanLongitude.innerHTML = "N/A";
              newSpanLongitude.id="LongitudeSpan";
              newSpanLongitude.className="form-label text-align-left padding-left-10";

              divDownTrack.append(newLabelDownTrack);
              divDownTrack.append(newInputDownTrack);
              divDownTrack.append(newLabelDownTrackUnit);

              //Longitude element and Down Track UI element are at the same row.
              divDownTrack.append(newLabelLongitudeTitle);
              divDownTrack.append(newSpanLongitude);

              divContentColOne.append(divDownTrack);

              //Minimum Gap
              var newLabelMinGap=document.createElement('label');
              newLabelMinGap.innerHTML = "Minimum Gap";
              newLabelMinGap.id="minGapLabel";
              newLabelMinGap.className="form-label col-3 text-align-right";

              var newInputMinGap = document.createElement('input');
              newInputMinGap.name = 'MinGap';
              newInputMinGap.className="form-control  col-2";
              newInputMinGap.id = 'MinGap';
              newInputMinGap.type = 'number';

              var newLabelDownTrackUnit=document.createElement('label');
              newLabelDownTrackUnit.innerHTML = "(Meter)";
              newLabelDownTrackUnit.className="col-1";

              //GPS
              var newLabelGPSStatusTitle = document.createElement('label');
              newLabelGPSStatusTitle.innerHTML = "GPS status:";
              newLabelGPSStatusTitle.id="GPSStatusLabelTitle";
              newLabelGPSStatusTitle.className="form-label offset-1 text-align-left ";

              var newSpanGPSStatus = document.createElement('span');
              newSpanGPSStatus.innerHTML = "N/A";
              newSpanGPSStatus.id="GPSStatusSpan";
              newSpanGPSStatus.className="form-label  text-align-left padding-left-10";

              var divMinGap = document.createElement('div');
              divMinGap.className = 'form-group row';

              divMinGap.append(newLabelMinGap);
              divMinGap.append(newInputMinGap);
              divMinGap.append(newLabelDownTrackUnit);

              //GPS status element and Minimum Gap UI element are at the same row.
              divMinGap.append(newLabelGPSStatusTitle);
              divMinGap.append(newSpanGPSStatus);
              divContentColOne.append(divMinGap);

              //Advisory Speed
              var newLabelAdvisorySpeed=document.createElement('label');
              newLabelAdvisorySpeed.innerHTML = "Advisory Speed:";
              newLabelAdvisorySpeed.id="AdvisorySpeedLabel";
              newLabelAdvisorySpeed.className="form-label col-3 text-align-right";

              var newInputAdvisorySpeed = document.createElement('input');
              newInputAdvisorySpeed.name = 'AdvisorySpeed';
              newInputAdvisorySpeed.className="form-control col-2";
              newInputAdvisorySpeed.id = 'AdvisorySpeed';

              var newLabelAdvisorySpeedUnit=document.createElement('label');
              newLabelAdvisorySpeedUnit.innerHTML = "(MPH)";
              newLabelAdvisorySpeedUnit .className="col-1";

              var divAdvisorySpeed = document.createElement('div');
              divAdvisorySpeed.className = 'form-group row';

              divAdvisorySpeed.append(newLabelAdvisorySpeed);
              divAdvisorySpeed.append(newInputAdvisorySpeed);
              divAdvisorySpeed.append(newLabelAdvisorySpeedUnit);

              //Justification
              var newLabelJustificationTitle = document.createElement('label');
              newLabelJustificationTitle.innerHTML = "Justification:";
              newLabelJustificationTitle.id="JustificationLabelTitle";
              newLabelJustificationTitle.className="form-label  offset-1 text-align-left ";

              var newSpanJustification = document.createElement('span');
              newSpanJustification.innerHTML = "Move Over Law";
              newSpanJustification.id="JustificationSpan";
              newSpanJustification.className="form-label  font-strong text-align-left padding-left-10";

              //Advisory speed element and Justification UI element are at the same row.
              divAdvisorySpeed.append(newLabelJustificationTitle);
              divAdvisorySpeed.append(newSpanJustification);
              divContentColOne.append(divAdvisorySpeed);

              //broadcasting frequency
              var divBlankCol6 = document.createElement('div');
              divBlankCol6.className = "col-6 margin-bottom-50px";

              //Broadcasting Frequency
              var newLabelBroadcastingFrequencyTitle = document.createElement('label');
              newLabelBroadcastingFrequencyTitle.innerHTML = "BroadCasting Frequency:";
              newLabelBroadcastingFrequencyTitle.id="BroadcastingFrequencyLabelTitle";
              newLabelBroadcastingFrequencyTitle.className="form-label hide offset-1 text-align-left ";

              var newLabelBroadcastingFrequency= document.createElement('span');
              newLabelBroadcastingFrequency.innerHTML = "N/A";
              newLabelBroadcastingFrequency.id="BroadcastingFrequencySpan";
              newLabelBroadcastingFrequency.className="form-label hide font-strong text-align-left padding-left-10";

              var divActionRowBroadcastingFrequency = document.createElement('div');
              divActionRowBroadcastingFrequency.className = 'form-group row BroadcastingFrequencyWrapper';
              divActionRowBroadcastingFrequency.append(divBlankCol6);
              divActionRowBroadcastingFrequency.append(newLabelBroadcastingFrequencyTitle);
              divActionRowBroadcastingFrequency.append(newLabelBroadcastingFrequency);
              divContentColOne.append(divActionRowBroadcastingFrequency);


              var divActionBtnWrapper = document.createElement('div');
              divActionBtnWrapper.className = 'form-group row actionWrapper';

              var divActionBtn = document.createElement('div');
              divActionBtn.className = 'offset-3 col-2';

              var actionBtn = document.createElement('input');
              actionBtn.id = "eventBroadcastBtn";
              actionBtn.type = "button";
              actionBtn.value = "Start Broadcast";
              actionBtn.className = 'btn btn-success btn-lg';
              actionBtn.addEventListener("click",function(){
                   console.log($(this).val());
                    if($(this).val().toLowerCase().includes('start'))
                    {
                       //send request to start broadcasting event
                       send_start_broadcasting_request(this);
                    }
                    else{
                        //send request to start broadcasting event
                       send_stop_broadcasting_request(this);
                    }
              });

              divActionBtn.append(actionBtn);
              divActionBtnWrapper.append(divActionBtn);
              divContentColOne.append(divActionBtnWrapper);

              $(this.element).append(divContentColOne);

              /***
                Top Col - 2 : Notes
              **/
              var fieldsetNote = document.createElement('fieldset');
              fieldsetNote.className = "notes row";
              var legendNote = document.createElement('legend');
              legendNote.innerHTML = "Notes";
              legendNote.className="text-align-left font-strong";
              fieldsetNote.append(legendNote);

              var imgNotePoliceCar = document.createElement('img');
              imgNotePoliceCar.name ="policeCar";
              imgNotePoliceCar.id="policeCarImg";
              imgNotePoliceCar.title="Police Car";
              imgNotePoliceCar.src="../../images/police_car.PNG";
              imgNotePoliceCar.className="img-notes";
              fieldsetNote.append(imgNotePoliceCar);

              var newSpanNoteDescription = document.createElement('span');
              newSpanNoteDescription.innerHTML = "This screen creates as in-field geo-fence around the vehicle.";
              newSpanNoteDescription.id="noteDescriptionSpan";
              newSpanNoteDescription.className="form-label margin-right-100px text-align-left font-strong block margin-bottom-20px";
              fieldsetNote.append(newSpanNoteDescription);

              var newSpanNoteUpTrack = document.createElement('span');
              newSpanNoteUpTrack.innerHTML = "Up Track: Distance from the center of the vehicle (Taheo) to the beginning of geofence.";
              newSpanNoteUpTrack.id="noteUpTrackSpan";
              newSpanNoteUpTrack.className="form-label  text-align-left block margin-bottom-20px";
              fieldsetNote.append(newSpanNoteUpTrack);

              var newSpanNoteDownTrack = document.createElement('span');
              newSpanNoteDownTrack.innerHTML = "Down Track: Distance between center of the vehicle (Taheo) to the center of the geofence.";
              newSpanNoteDownTrack.id="noteDownTrackSpan";
              newSpanNoteDownTrack.className="form-label text-align-left block margin-bottom-20px";
              fieldsetNote.append(newSpanNoteDownTrack);

              var newSpanNoteMinGap = document.createElement('span');
              newSpanNoteMinGap.innerHTML = "Minimum Gap: Road vehicles minimum following distance.";
              newSpanNoteMinGap.id="noteMinGapSpan";
              newSpanNoteMinGap.className="form-label text-align-left block margin-bottom-20px";
              fieldsetNote.append(newSpanNoteMinGap);

              var newSpanNoteAdvisorySpeed = document.createElement('span');
              newSpanNoteAdvisorySpeed.innerHTML = "Advisory Speed: Recommended speed within Geofence. ";
              newSpanNoteAdvisorySpeed.id="noteAdvisorySpeedSpan";
              newSpanNoteAdvisorySpeed.className="form-label text-align-left block ";
              fieldsetNote.append(newSpanNoteAdvisorySpeed);

              divNoteColTwo.append(fieldsetNote);

              $(this.element).append(divNoteColTwo);

              //Once the UI elements are loaded. Update the form values with session variables values
              checkSessionVariables();

            console.log("Event Management page is created");
        },
        subscribe_gps_common_fix: function(){
            subscribe_gps_common_fix();
        },
        _destroy: function () {
            this.element.empty();
            this._super();
            console.log("event management configuration page is destroyed");
        }
    });//CarmaJS.

    var loadCustomWidget = function (container) {
        //create management page
        container.eventManagement();
        container.eventManagement("subscribe_gps_common_fix", null);
    };

    //*** Public API  ***
    return {
        loadCustomWidget: loadCustomWidget
    };
})();