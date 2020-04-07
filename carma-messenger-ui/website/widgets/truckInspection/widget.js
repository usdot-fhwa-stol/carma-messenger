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

    var SafetyLogListNotify = function () {

        listenerSaftyLogList = new ROSLIB.Topic({
            ros: ros,
            name: '/cav_truck_identified',//cav_truck_identified,
            messageType: 'std_msgs/String'// 'std_msgs/String.msg'
        });
        try{
                console.log("/cav_truck_identified topic called");
                listenerSaftyLogList.subscribe(function (message) {
                    console.log("/cav_truck_identified message.data: "+ message.data);                 
                
                    if (message.data != null && message.data != 'undefined' && message.data.length > 0){
                        var button = document.getElementById('LogbtnId');
                        var label =document.getElementById('lblLogbtnId');
                        if(button != null){                       
                            button.style.display='';  
                            button.name = message.data;
                        }                    
                        if(label != null){  
                            label.innerHTML = 'Automated Vehicle VIN: '+message.data+' is within range.';                       
                            label.style.display='';
                        }   
                        
                        if(document.getElementById('NoMsgAvailableId') != null)               
                            document.getElementById('NoMsgAvailableId').style.display='none'; 
                        
                        //refresh the log list information every 10 seconds
                        setTimeout(() => {  
                            document.getElementById('LogbtnId').style.display='none';
                            document.getElementById('lblLogbtnId').style.display='none';        
                            document.getElementById('NoMsgAvailableId').style.display='';
                            console.log("log list is hide after "+CarmaJS.Config.getRefreshInterval()+" seconds");
                        }, CarmaJS.Config.getRefreshInterval()*1000);
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
            name: '/truck_safety_info',//truck_safety_info,
            messageType: 'std_msgs/String'// 'rostopic pub /ui_platoon_vehicle_info std_msgs/String '{data: this is my instructions}'
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
        var arrow = document.getElementById("Messenger_back_arrow");
        arrow.style.display='inline-block'; 

        //Create widget for log detail
        CreateTruckInspectionSafetyLogPerVINWidget($("#divWidgetArea"));
        //Display log detail info on the detail widget
        var rawStr = messageData;
        var rawArr = rawStr.split(",");
        console.log("Current Millseconds: "+new Date().getTime());
        rawArr.forEach(element => {
            var rawElement = element.split(":");
            console.log(rawElement[0]+" : "+ rawElement[1]);
            var key = rawElement[0].trim().toUpperCase();
            var value=rawElement[1].trim();
            if(value==null || value.length==0 || value== ""){
                value="--";
            }
            if(key=="TIMESTAMP"){
                var logTimetamp = new Date(parseInt(value, 10));
                document.getElementById('SystemDateTimeSpan').innerText = logTimetamp.getFullYear() 
                                                                        + "-" + ((logTimetamp.getMonth() + 1)<=9?"0"+(logTimetamp.getMonth() + 1):(logTimetamp.getMonth() + 1)) 
                                                                        + "-" + (logTimetamp.getDate()<=9?"0"+logTimetamp.getDate():logTimetamp.getDate()) + " " 
                                                                        + (logTimetamp.getHours()+3) + ":" + (logTimetamp.getMinutes()<=9?"0"+logTimetamp.getMinutes():logTimetamp.getMinutes()) + ":" + logTimetamp.getSeconds();
                document.getElementById('SystemDateTimeSpan').style.display="";
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
                document.getElementById('LicenseTextId')!=null ? document.getElementById('LicenseTextId').innerHTML=value:"";
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
                document.getElementById('ISSTextId')!=null ? document.getElementById('ISSTextId').innerHTML=value:"";
            }
            if(key=="ADS_SOFTWARE_VERSION"){
                document.getElementById('ADSSoftwareVersionTextId')!=null ? document.getElementById('ADSSoftwareVersionTextId').innerHTML=value:"";
            }
            if(key=="PRE_TRIP_ADS_HEALTH_CHECK" && value.toLowerCase().trim() == "green" && document.getElementById('PreADSHealthCheckTextId')!=null){
                document.getElementById('PreADSHealthCheckTextId').classList.replace("RedBkColor","GreenBkColor");
            }
            if(key=="ADS_STATUS" && value.toLowerCase().trim() == "green" && document.getElementById('ADSHealthStatusTextId') != null){
                document.getElementById('ADSHealthStatusTextId').classList.replace("RedBkColor","GreenBkColor");
            }   
        });
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
            var VINFrameStr = createDivFramePanelText('VINFrameId', 'VINFrame', 'VINPanelId', 'VINPanel', 'VIN', 'VINTextId', 'VINText','no data');
            var CarrierNameFrame = createDivFramePanelText('CarrierNameFrameId', 'CarrierNameFrame', 'CarrierNamePanelId', 'CarrierNamePanel', 'Carrier Name', 'CarrierNameTextId', 'CarrierNameText','no data');
            var InspectionFrame = createDivFramePanelText('InspectionFrameId', 'InspectionFrame', 'InspectionPanelId', 'InspectionPanel', 'Last State Inspection', 'InspectionTextId', 'InspectionText','no data');
            var CalibrationFrame = createDivFramePanelText('CalibrationFrameId', 'CalibrationFrame', 'CalibrationPanelId', 'CalibrationPanel', 'Last ADS Calibration', 'CalibrationTextId', 'CalibrationText','no data');
            var LicenseFrame = createDivFramePanelText('LicenseFrameId', 'LicenseFrame', 'LicensePanelId', 'LicensePanel', 'License', 'LicenseTextId', 'LicenseText','no data');
            var WeightFrame = createDivFramePanelText('WeightFrameId', 'WeightFrame', 'WeightPanelId', 'WeightPanel', 'Weight(lbs.)', 'WeightTextId', 'WeightText','no data');
            var CarrierFrame = createDivFramePanelText('CarrierFrameId', 'CarrierFrame', 'CarrierPanelId', 'CarrierPanel', 'Carrier ID', 'CarrierTextId', 'CarrierText','no data');
            var PermitFrame = createDivFramePanelText('PermitFrameId', 'PermitFrame', 'PermitPanelId', 'PermitPanel', 'Permit Required', 'PermitTextId', 'PermitText','no data');
            var ISSFrame = createDivFramePanelText('ISSFrameId', 'ISSFrame', 'ISSPanelId', 'ISSPanel', 'ISS', 'ISSTextId', 'ISSText','--');
            var PreADSHealthCheckFrame = createDivFramePanelText('PreADSHealthCheckFrameId', 'PreADSHealthCheckFrame', 'PreADSHealthCheckPanelId', 'PreADSHealthCheckPanel', 'ADS Pre-Trip Check', 'PreADSHealthCheckTextId', 'PreADSHealthCheckText RedBkColor');
            var ADSHealthStatusFrame = createDivFramePanelText('ADSHealthStatusFrameId', 'ADSHealthStatusFrame', 'ADSHealthStatusPanelId', 'ADSHealthStatusPanel', 'ADS Health Status', 'ADSHealthStatusTextId', 'ADSHealthStatusText RedBkColor');
            var ADSSoftwareVersionFrame = createDivFramePanelText('ADSSoftwareVersionFrameId', 'ADSSoftwareVersionFrame', 'ADSSoftwareVersionPanelId', 'ADSSoftwareVersionPanel', 'ADS Software Version', 'ADSSoftwareVersionTextId', 'ADSSoftwareVersionText','no data');
            var truckImage = createImgFramePanelText('truckImgFrameId', 'truckImgFrame', 'truckImgPanelId', 'truckImgPanel', 'truckImgId', 'truckImg', '../../images/truck_unavailable.jpg', 'Truck Image is unavailable');
            var aceIcon = createImgFramePanelText('aceIconFrameId_detailPg', 'aceIconFrame_detailPg', 'taceIconPanelId_detailPg', 'aceIconPanel_detailPg', 'aceIconTextId_detailPg', 'aceIcon_detailPg', '../../images/ace_icon.png', 'Image is unavailable');
            var logSpan = createSpan('LogSpanId_detailPg', 'logSpan_detailPg', 'Safety Log');
            var leftDiv = createDiv('leftDivId', 'leftDivCSS');
            var rightDiv = createDiv('rightDivId', 'rightDivCSS');
            leftDiv.append(VINFrameStr);
            leftDiv.append(LicenseFrame);
            leftDiv.append(WeightFrame);
            leftDiv.append(CarrierNameFrame);
            leftDiv.append(CarrierFrame);
            leftDiv.append(PermitFrame);
            leftDiv.append(InspectionFrame);
            leftDiv.append(ISSFrame);
            leftDiv.append(PreADSHealthCheckFrame);
            leftDiv.append(ADSHealthStatusFrame);
            leftDiv.append(CalibrationFrame);
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
              newLabel.innerHTML += 'No data';
              newLabel.style.display='none';
              newInput.style.display='none';
              divBtnLabelUnit.appendChild(newLabel);
              divBtnLabelUnit.appendChild(newInput);
  
              //create ACE logo
              var aceIcon = createImgFramePanelText('aceIconFrameId', 'aceIconFrame', 'taceIconPanelId', 'aceIconPanel', 'aceIconTextId', 'aceIcon', '../../images/ace_icon.png', 'Image is unavailable');      
              
              //Create Unavailable message
              var NoMsgAvailable=createDiv("NoMsgAvailableId","NoMsgAvailable","No message availabe");
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