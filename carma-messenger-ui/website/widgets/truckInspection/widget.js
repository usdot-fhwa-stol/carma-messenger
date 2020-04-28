/***
* Create a unique namespace for each plugin widget to minimize collision of same name variables or functions.
***/
CarmaJS.registerNamespace("CarmaJS.WidgetFramework.truckInspection");

CarmaJS.WidgetFramework.truckInspection = (function () {

    //*** Private Variables ***
    //Listeners
    //*** Widget Install Folder ***
    //Currently the URL path from document or window are pointing to the page, not the actual folder location.
    //Therefore this needs to be hardcoded.
    var installfoldername = 'widgets/truckInspection/';
    var isTimeoutSet = false;

    var SafetyLogListNotify = function () {

        listenerSaftyLogList = new ROSLIB.Topic({
            ros: ros,
            name: '/cav_truck_identified',
            messageType: 'std_msgs/String'
        });
        try{
                console.log("/cav_truck_identified topic called");
                listenerSaftyLogList.subscribe(function (message) {
                    console.log("/cav_truck_identified message.data: "+ message.data);                 
                
                    if (message.data != null && message.data != 'undefined' && message.data.length > 0){
                        let rawStr = message.data;
                        let rawArr = rawStr.split(",");
                        let state = "";
                        let license_plate = "";
                        let vin_number = "";
                        rawArr.forEach(element => {
                            let rawElement = element.split(":");
                            let key = rawElement[0].trim().toUpperCase();
                            let value=rawElement[1].trim().toUpperCase();
                            console.log(value);
                            if(key == "VIN_NUMBER"){
                                vin_number=value;
                            }
                            if(key=="LICENSE_PLATE"){
                                license_plate=value;
                            } 
                            if(key=="STATE_SHORT_NAME"){
                                state=value;
                            }               
                        });
                        var button = document.getElementById('LogbtnId');
                        var label =document.getElementById('lblLogbtnId');
                        if(button != null){                       
                            button.style.display='';  
                            button.name = message.data;
                        }                    
                        if(label != null){  
                            label.innerHTML = 'Automated Vehicle is within range.<br>'+ "VIN: " + vin_number+", License Plate: "+ state+" " + license_plate;                       
                            label.style.display='';
                        }   
                        
                        if(document.getElementById('NoMsgAvailableId') != null)               
                            document.getElementById('NoMsgAvailableId').style.display='none'; 
                        
                        //refresh the log list information once and only once  every 30 seconds
                        if(!isTimeoutSet){
                            isTimeoutSet=true;
                            console.log("isTimeoutSet: "+isTimeoutSet);
                            setTimeout(() => {     
                                let LogBtn=document.getElementById('LogbtnId'); 
                                let LogLbl=document.getElementById('lblLogbtnId');
                                let NoMsgAvailableLbl = document.getElementById('NoMsgAvailableId'); 
                                if(LogBtn!=null)
                                    LogBtn.style.display='none';
                                if(LogLbl!=null)
                                    LogLbl.style.display='none'; 
                                if(NoMsgAvailableLbl!=null)
                                    NoMsgAvailableLbl.style.display='';                                       
                                //once timeout is called, reset isTimeout
                                isTimeoutSet=false;
                                console.log("log list is hide after "+CarmaJS.Config.getRefreshInterval()+" seconds." );
                            }, CarmaJS.Config.getRefreshInterval()*1000);
                        }
                       
                    }
            });
        }catch(ex){
            console.error("error");
        }
    };
    var SubscribeToSafetyLog = function(){
        console.log("function subscribeToSafetyLog: called");
        listenerSaftyLogPerVIN = new ROSLIB.Topic({
            ros: ros,
            name: '/truck_safety_info',
            messageType: 'std_msgs/String'
        });       
       
        listenerSaftyLogPerVIN.subscribe(function (message) {
            console.log("function subscribeToSafetyLog: "+ message);
            if (message.data != null && message.data != 'undefined') {
                ShowSafetyLogPerVIN(message.data);
            }           
        });
    }

    var ShowSafetyLogPerVIN = function (messageData) {    
        $('#divWidgetArea').empty();
        //enable back button
        let arrow = document.getElementById("Messenger_back_arrow");
        arrow.style.display='inline-block'; 

        //Create widget for log detail
        CreateTruckInspectionSafetyLogPerVINWidget($("#divWidgetArea"));
        //Display log detail info on the detail widget
        let rawStr = messageData;
        let rawArr = rawStr.split(",");
        let isEngaged = false;
        //Check if Auto status is engaged or not engaged
        rawArr.forEach(element => {
            let rawElement = element.split(":");
            let key = rawElement[0].trim().toUpperCase();
            let value=rawElement[1].trim().toUpperCase();
            if(key == "ADS_AUTO_STATUS" && value == "ENGAGED"){
                isEngaged=true;
            }                
        });

        //remove all "ADS Health Status" color css
        $("#ADSHealthStatusTextId").removeClass("BlackBkColor RedBkColor GreenBkColor OrangeBkColor GreyBkColor GreenBkColor");
        if(isEngaged){
            $('#ADSHealthStatusTextId').addClass("GreyBkColor"); //NOT-READY: Grey Color
        }
        else{                   
           $('#ADSHealthStatusTextId').addClass("BlackBkColor"); //SHUTDOWN : Black Color
        }

        //Set DOM element data
        let state="";
        let license_plate="";
        rawArr.forEach(element => {
            let rawElement = element.split(":");
            let key = rawElement[0].trim().toUpperCase();
            let value=rawElement[1].trim();
            if(value==null || value.length==0 || value== ""){
                value="--";
            }
            if(key=="TIMESTAMP"){
                let logTimetamp = new Date(parseInt(value, 10));
                if(document.getElementById('SystemDateTimeSpan')!=null)
                {
                    document.getElementById('SystemDateTimeSpan').innerText = logTimetamp.getFullYear() 
                                                                            + "-" + ((logTimetamp.getMonth() + 1)<=9?"0"+(logTimetamp.getMonth() + 1):(logTimetamp.getMonth() + 1)) 
                                                                            + "-" + (logTimetamp.getDate()<=9?"0"+logTimetamp.getDate():logTimetamp.getDate()) + " " 
                                                                            + (logTimetamp.getHours()+3) + ":" + (logTimetamp.getMinutes()<=9?"0"+logTimetamp.getMinutes():logTimetamp.getMinutes()) + ":" + logTimetamp.getSeconds();
                    document.getElementById('SystemDateTimeSpan').style.display="";
                }
            }
            if(key=="VIN_NUMBER"){
                document.getElementById('VINTextId')!=null ? document.getElementById('VINTextId').innerHTML=value:"";
                document.getElementById('truckImgId')!=null ? document.getElementById('truckImgId').src="../../images/truck_"+value+".jpg":"";                        
            }
            if(key=="CARRIER_NAME"){
                document.getElementById('CarrierNameTextId')!=null ? document.getElementById('CarrierNameTextId').innerHTML=value:"";
            }
            if(key=="DATE_OF_LAST_STATE_INSPECTION"){
                value=value.replace(/\./g,'-');
                document.getElementById('InspectionTextId')!=null ? document.getElementById('InspectionTextId').innerHTML=value:"";
            }
            if(key=="DATE_OF_LAST_ADS_CALIBRATION"){
                value=value.replace(/\./g,'-');
                document.getElementById('CalibrationTextId')!=null ? document.getElementById('CalibrationTextId').innerHTML=value:"";
            }
            if(key=="LICENSE_PLATE"){                
                license_plate=value;
            } 
            if(key=="STATE_SHORT_NAME"){                
                state=value;
            }
            if(key=="WEIGHT"){
                    value=value.toString().replace(/(\d)(?=(\d{3})+(?!\d))/g,'$1,'); 
                    //value=value.length>3?value[0]+","+value.substring(1,value.length):value;
                document.getElementById('WeightTextId')!=null ? document.getElementById('WeightTextId').innerHTML=value:"";
            }
            if(key=="CARRIER_ID"){
                document.getElementById('CarrierTextId')!=null ? document.getElementById('CarrierTextId').innerHTML=value:"";
            }
            if(key=="PERMIT_REQUIRED"){
                value = (value ==0 ? "No":"Yes");
                document.getElementById('PermitTextId')!=null ? document.getElementById('PermitTextId').innerHTML=value:"";
            }
            if(key=="ISS_SCORE"){
                value = parseInt(value,10);
                let color="";

                if(value>0 && value<50)
                    color="limegreen";
                else if(value>=50 && value<75)
                    color="orange";
                else if(value>=75 && value <=100)
                    color="red";
                else{
                    //Do Nothing
                }
                console.log("color:" +color);
                document.getElementById('ISSTextId')!=null ? document.getElementById('ISSTextId').innerHTML=value:"";
                document.getElementById('ISSTextId')!=null ? document.getElementById('ISSTextId').style.color=color:"";
            }
            if(key=="ADS_SOFTWARE_VERSION"){
                document.getElementById('ADSSoftwareVersionTextId')!=null ? document.getElementById('ADSSoftwareVersionTextId').innerHTML=value:"";
            }
            if(key=="PRE_TRIP_ADS_HEALTH_CHECK" && value.toLowerCase().trim() == "green" && document.getElementById('PreADSHealthCheckTextId')!=null){
                document.getElementById('PreADSHealthCheckTextId').classList.replace("RedBkColor","GreenBkColor");
            }              
            if(key=="ADS_AUTO_STATUS"  && document.getElementById('ADSAutoStatusTextIdImg') != null){
                console.log("ADS_AUTO_STATUS: "+value);
                if(value.toUpperCase() == "ENGAGED" || value == "engaged"){
                    document.getElementById('ADSAutoStatusTextIdImg')!=null ? document.getElementById('ADSAutoStatusTextIdImg').src="../../images/ads_auto_status_wheel_green.png":"";
                }
                else{
                    document.getElementById('ADSAutoStatusTextIdImg')!=null ? document.getElementById('ADSAutoStatusTextIdImg').src="../../images/ads_auto_status_wheel_grey.png":"";
                }                
            }
            if(key=="ADS_HEALTH_STATUS" && isEngaged && document.getElementById('ADSHealthStatusTextId') != null){
                console.log("ADS_HEALTH: "+value);
                //remove all "ADS Health Status" color css
                $("#ADSHealthStatusTextId").removeClass("BlackBkColor RedBkColor GreenBkColor OrangeBkColor GreyBkColor GreenBkColor");
                switch(value){
                    case "1": //CAUTION - Yellow
                         $("#ADSHealthStatusTextId").addClass("YellowBkColor");
                         break;
                    case "2": //WARNING - Orange
                         $("#ADSHealthStatusTextId").addClass("OrangeBkColor");
                         break;
                    case "3": //FATAL - Red
                        $("#ADSHealthStatusTextId").addClass("RedBkColor");
                        break;
                    case "4": //NOT_READY - Grey
                        $("#ADSHealthStatusTextId").addClass("GreyBkColor");
                        break;
                    case "5": //DRIVERS_READY - Green
                        $("#ADSHealthStatusTextId").addClass("GreenBkColor");                        
                        break;  
                    case "6": //SHUTDOWN - Black   
                        $("#ADSHealthStatusTextId").addClass("BlackBkColor");
                        break;
                }
            }
             
        }); //End of DOM element data iteration

        //Supplement data for DOM element: state, license_plate
        document.getElementById('LicenseTextId')!=null ? document.getElementById('LicenseTextId').innerHTML=(state +" " + license_plate):"";

    }      

    function sendTruckInspectionReq(btn) {    
        console.log("send truck inspection request");
        var sendTruckInspection = new ROSLIB.Service({
            ros: ros,
            name: '/send_inspection_request',
            serviceType: 'std_srvs/Trigger'
        });        
        var request = new ROSLIB.ServiceRequest({});
        try{       
            sendTruckInspection.callService(request, function(result) {
                console.log("send truck inspection request . Result messageg is: "+result.success);           
                if(!result.success){                            
                        alert("Log is not available, please try again in 5 seconds");
                        setTimeout(() => {  
                            if(btn != "undefined" && btn.style != "undefined"){
                            btn.disabled=false;  
                            btn.style.color='white';  
                            btn.value="Request Safety Log";
                            }                
                        }, 5000);
                }
                else
                { 
                    SubscribeToSafetyLog();
                }
                
            }); 
        }catch(ex){
            alert("Log is not available, please try again in 10 seconds");
            console.error("send truck inspection request error: " + ex.message);
        }        
    }

    var CreateTruckInspectionSafetyLogPerVINWidget = function(container){
            var VINFrameStr = createDivFramePanelText('VINFrameId', 'VINFrame', 'VINPanelId', 'VINPanel', 'VIN', 'VINTextId', 'VINText','--');
            var CarrierNameFrame = createDivFramePanelText('CarrierNameFrameId', 'CarrierNameFrame', 'CarrierNamePanelId', 'CarrierNamePanel', 'Carrier Name', 'CarrierNameTextId', 'CarrierNameText','--');
            var InspectionFrame = createDivFramePanelText('InspectionFrameId', 'InspectionFrame', 'InspectionPanelId', 'InspectionPanel', 'Last State Inspection', 'InspectionTextId', 'InspectionText','--');
            var CalibrationFrame = createDivFramePanelText('CalibrationFrameId', 'CalibrationFrame', 'CalibrationPanelId', 'CalibrationPanel', 'Last ADS Calibration', 'CalibrationTextId', 'CalibrationText','--');
            var LicenseFrame = createDivFramePanelText('LicenseFrameId', 'LicenseFrame', 'LicensePanelId', 'LicensePanel', 'License Plate', 'LicenseTextId', 'LicenseText','--');
            //var StateFrame = createDivFramePanelText('StateFrameId', 'StateFrame', 'StatePanelId', 'StatePanel', 'STATE', 'StateTextId', 'StateText','--');
            var CarrierFrame = createDivFramePanelText('CarrierFrameId', 'CarrierFrame', 'CarrierPanelId', 'CarrierPanel', 'Carrier ID', 'CarrierTextId', 'CarrierText','--');
            var PermitFrame = createDivFramePanelText('PermitFrameId', 'PermitFrame', 'PermitPanelId', 'PermitPanel', 'Permit Req', 'PermitTextId', 'PermitText','--');
            var ISSFrame = createDivFramePanelText('ISSFrameId', 'ISSFrame', 'ISSPanelId', 'ISSPanel', 'ISS', 'ISSTextId', 'ISSText','--');
            var PreADSHealthCheckFrame = createDivFramePanelText('PreADSHealthCheckFrameId', 'PreADSHealthCheckFrame', 'PreADSHealthCheckPanelId', 'PreADSHealthCheckPanel', 'ADS Pre-Trip Check', 'PreADSHealthCheckTextId', 'PreADSHealthCheckText RedBkColor');
            var ADSHealthStatusFrame = createDivFramePanelText('ADSHealthStatusFrameId', 'ADSHealthStatusFrame', 'ADSHealthStatusPanelId', 'ADSHealthStatusPanel', 'ADS Health Status', 'ADSHealthStatusTextId', 'ADSHealthStatusText RedBkColor');
            var ADSAutoStatusFrame = createDivFramePanelText('ADSAutoStatusFrameId', 'ADSAutoStatusFrame', 'ADSAutoStatusPanelId', 'ADSAutoStatusPanel', 'ADS AUTO Status', 'ADSAutoStatusTextId', 'ADSAutoStatusText','../../images/ads_auto_status_wheel_grey.png');
            var ADSSoftwareVersionFrame = createDivFramePanelText('ADSSoftwareVersionFrameId', 'ADSSoftwareVersionFrame', 'ADSSoftwareVersionPanelId', 'ADSSoftwareVersionPanel', 'ADS Software Version', 'ADSSoftwareVersionTextId', 'ADSSoftwareVersionText','--');
            var WeightFrame = createDivFramePanelText('WeightFrameId', 'WeightFrame', 'WeightPanelId', 'WeightPanel', 'Weight(lbs.)', 'WeightTextId', 'WeightText','--');
            var truckImage = createImgFramePanelText('truckImgFrameId', 'truckImgFrame', 'truckImgPanelId', 'truckImgPanel', 'truckImgId', 'truckImg', '../../images/truck_unavailable.jpg', 'Truck Image is unavailable');
            var aceIcon = createImgFramePanelText('aceIconFrameId_detailPg', 'aceIconFrame_detailPg', 'taceIconPanelId_detailPg', 'aceIconPanel_detailPg', 'aceIconTextId_detailPg', 'aceIcon_detailPg', '../../images/Ace_Color_Tagline.png', 'Image is unavailable');
            var logSpan = createSpan('LogSpanId_detailPg', 'logSpan_detailPg', 'Safety Log');
            var leftDiv = createDiv('leftDivId', 'leftDivCSS');
            var rightDiv = createDiv('rightDivId', 'rightDivCSS');
            leftDiv.append(VINFrameStr);
            leftDiv.append(LicenseFrame);
            //leftDiv.append(StateFrame);
            leftDiv.append(PermitFrame);
            leftDiv.append(CarrierNameFrame);
            leftDiv.append(CarrierFrame);
            leftDiv.append(ISSFrame);
            leftDiv.append(InspectionFrame);
            leftDiv.append(PreADSHealthCheckFrame);
            leftDiv.append(ADSHealthStatusFrame);
            leftDiv.append(ADSAutoStatusFrame);
            leftDiv.append(CalibrationFrame);
            leftDiv.append(WeightFrame);
            leftDiv.append(ADSSoftwareVersionFrame);

            $(container).append(leftDiv);

            rightDiv.appendChild(truckImage);
            rightDiv.appendChild(logSpan);
            rightDiv.appendChild(aceIcon);

            $(container).append(rightDiv);

            console.log("truckInspectionSafetyLogPerVIN element is created");
    }
    /***
       * Custom widgets using JQuery Widget Framework.
       * NOTE: that widget framework namespace can only be one level deep.
       * This should not colide with other widgets as loadCustomWidget will be calling private widgets.
       ***/

    $.widget("CarmaJS.truckInspectionSafetyLogList", {
        _create: function () {
              //create request button
              var divBtnLabelUnit=document.createElement('div');
              divBtnLabelUnit.className="divBtnLabelUnitLogbtnId";
              var newInput = document.createElement('input');
              newInput.type = 'button';
              newInput.name = 'btnName';
              newInput.id = 'LogbtnId';
              newInput.setAttribute('title', 'Request Safety Log');
              newInput.value='Request Safety Log';  
              newInput.onclick = function () { 
                  this.disabled=true;
                  this.value="Requested";        
                  sendTruckInspectionReq(this);    
              }; 
  
              //create request VIN label
              var newLabel = document.createElement('label');
              newLabel.id = 'lblLogbtnId';  
              newLabel.innerHTML += '--';
              newLabel.style.display='none';
              newInput.style.display='none';
              divBtnLabelUnit.appendChild(newLabel);
              divBtnLabelUnit.appendChild(newInput);
  
              //create ACE logo
              var aceIcon = createImgFramePanelText('aceIconFrameId', 'aceIconFrame', 'taceIconPanelId', 'aceIconPanel', 'aceIconTextId', 'aceIcon', '../../images/Ace_Color_Tagline.png', 'Image is unavailable');      
              
              //Create Unavailable message
              var NoMsgAvailable=createDiv("NoMsgAvailableId","NoMsgAvailable","No vehicles in range");
            $(this.element).append(NoMsgAvailable);
            $(this.element).append(divBtnLabelUnit);
            $(this.element).append(aceIcon);
            console.log("truckInspectionSafetyLogList element is created");
        },
        _destroy: function () {
            this.element.empty();
            this._super();
            console.log("truckInspectionSafetyLogList element is destroyed");
        },
        SafetyLogListNotify: function () {
            SafetyLogListNotify();
        }
    });//CarmaJS.truckInspectionSafetyLogList

    var loadCustomWidget = function (container) {
        //Safty Log List
        container.truckInspectionSafetyLogList();
        container.truckInspectionSafetyLogList("SafetyLogListNotify", null);        
    };

    //*** Public API  ***
    return {
        loadCustomWidget: loadCustomWidget
    };
})();