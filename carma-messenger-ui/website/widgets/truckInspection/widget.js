/***
* Create a unique namespace for each plugin widget to minimize collision of same name variables or functions.
***/
CarmaJS.registerNamespace("CarmaJS.WidgetFramework.truckInspection");

CarmaJS.WidgetFramework.truckInspection = (function () {

    //*** Private Variables ***
    var widgetName;
    //Listeners

    //*** Widget Install Folder ***
    //Currently the URL path from document or window are pointing to the page, not the actual folder location.
    //Therefore this needs to be hardcoded.
    //TODO: However, this could be set by widgetfw based on final install folder naming convention.
    var installfoldername = 'widgets/truckInspection/';

    //*** Functions ***
    /***
    * Listen to safty log topics
    * Assumes that ROSLIB object has been initalized.
    ***/

   /**** Launch roscore
    * 
    * roscore
    */

    /**** Launch rosbridge
     * 
     * roslaunch rosbridge_server rosbridge_websocket.launch 
    */

    /*** Launch plugin
    * 
    * roslaunch rosbridge_server rosbridge_websocket.launch 
    * OR
    * rosrun truck_inspection_plugin truck_inspection_plugin  
    ****/
   
    //rostopic pub /cav_truck_identified std_msgs/String "1FUJGHDV0CLBP8834"
    //rosservice call /send_inspection_request "{}" 
    //rostopic echo /cav_truck_identified 

    /***
     * Load MobilityOperation
     * 
     * 
rostopic pub /mobility_operation_inbound cav_msgs/MobilityOperation "header{sender_id: '', recipient_id: '', sender_bsm_id: '', plan_id: '', timestamp: 0}
strategy: 'TruckInspection'
strategy_params: 'VIN:2UJGHDV0CLBP8834,CarrierName:FMCSA Tech Division,Inspection:2021-03-30,Calibration:2020-03-30,License:DOT-2003,Weight:12000,Carrier:DOT-2,Permit:No,ISS:25,PreADS:good,ADSStatus:good,ADSSoftwareVersion:CARMA Platform v3.3.4'"

rostopic pub /mobility_operation_inbound cav_msgs/MobilityOration "header: {sender_id: '', recipient_id: '', sender_bsm_id: '', plan_id: '', timestamp: 0}
strategy: 'TruckInspection'
strategy_params: 'VIN:2UJGHDV0CLBP8834'"
        * 
     */
    var SafetyLogListNotify = function () {

        listenerSaftyLogList = new ROSLIB.Topic({
            ros: ros,
            name: '/cav_truck_identified',//cav_truck_identified,
            messageType: 'std_msgs/String'// 'std_msgs/String.msg'
        });
        try{
            listenerSaftyLogList.subscribe(function (message) {
                console.log("/cav_truck_identified message.data: "+ message.data);
                if (message.data != null && message.data != 'undefined' && message.data.length > 0){
                    var button = document.getElementById('btnbtnId');
                    var label =document.getElementById('lblbtnId');
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
                }
            });
        }catch(ex){
            console.error("error");
        }
    };
    var SubscribeToSafetyLog = function(){
        listenerSaftyLogPerVIN = new ROSLIB.Topic({
            ros: ros,
            name: '/truck_safety_info',//truck_safety_info,
            messageType: 'std_msgs/String'// 'rostopic pub /ui_platoon_vehicle_info std_msgs/String '{data: this is my instructions}'
        });
         
       
        listenerSaftyLogPerVIN.subscribe(function (message) {
        console.log("function subscribeToSafetyLog: "+ message);
       // var arrow = document.getElementById("Messenger_back_arrow");
       // arrow.style.display='inline-block'; 
       // CarmaJS.WidgetFramework.closeWidgets();
       // CarmaJS.WidgetFramework.listWidgetOptions('detail',message); 
           
        });
    }

    var ShowSafetyLogPerVIN = function (message) {
        //SubscribeToSafetyLog();
        // var arrow = document.getElementById("Messenger_back_arrow");
        // arrow.style.display='inline-block'; 
        // CarmaJS.WidgetFramework.closeWidgets();
        // CarmaJS.WidgetFramework.listWidgetOptions('detail'); 
        listenerSaftyLogPerVIN = new ROSLIB.Topic({
            ros: ros,
            name: '/truck_safety_info',//truck_safety_info,
            messageType: 'std_msgs/String'// 'rostopic pub /ui_platoon_vehicle_info std_msgs/String '{data: this is my instructions}'
        });
         
       
        listenerSaftyLogPerVIN.subscribe(function (message) {
            console.log("function subscribeToSafetyLog: "+ message);   
            console.log("function ShowSafetyLogPerVIN:  "+ message); 
            if (message.data != null && message.data != 'undefined') {
                console.log(message.data);                
                var rawStr = message.data;
                var rawArr = rawStr.split(",");
                console.log("Current Millseconds: "+new Date().getTime())
                rawArr.forEach(element => {
                    var rawElement = element.split(":");
                    console.log(rawElement[0]+" : "+ rawElement[1]);
                    var key = rawElement[0].trim().toUpperCase();
                    var value=rawElement[1].trim();
                    if(value==null || value.length==0 || value== ""){
                        value="no data";
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
                        document.getElementById('InspectionTextId')!=null ? document.getElementById('InspectionTextId').innerHTML=value:"";
                    }
                    if(key=="DATE_OF_LAST_ADS_CALIBRATION"){
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
        });
        /***
        //{VIN: 2FUJGHDV0CLBP8834,CarrierName: FMCSA Tech Division,Inspection: 2021-03-30,Calibration: 2020-03-30,License: DOT-2003,Weight: 120000,Carrier: DOT 2,Permit: No,ISS: 25,PreADS: good,ADSStatus:good,ADSSoftwareVersion: CARMA Platform v3.3.1}
        //rostopic pub /truck_safety_info std_msgs/String "data{'VIN'_'2UJGHDV0CLBP8834'#'CarrierName'_'FMCSA Tech Division'#'Inspection'_'2021-03-30'#'Calibration'_'2020-03-30'#'License'_'DOT-2003'#'Weight'_'12000'#'Carrier'_'DOT-2'#'Permit'_'No'#'ISS'_'25'#'PreADS'_'good'#'ADSStatus'_'good'#'ADSSoftwareVersion'_'CARMA Platform v3.3.1'}"
        //rostopic pub /truck_safety_info std_msgs/String "data:'{'VIN':'2UJGHDV0CLBP8834','CarrierName':'FMCSA Tech Division','Inspection':'2021-03-30','Calibration':'2020-03-30','License':'DOT-2003','Weight':'12000','Carrier':'DOT-2','Permit':'No','ISS':'25','PreADS':'good','ADSStatus':'good','ADSSoftwareVersion':'CARMA Platform v3.3.1'}'"
        //rostopic pub /truck_safety_info std_msgs/String "data:{'VIN':'2UJGHDV0CLBP8834','CarrierName':'FMCSA Tech Division','Inspection':'2021-03-30','Calibration':'2020-03-30','License':'DOT-2003','Weight':'12000','Carrier':'DOT-2','Permit':'No','ISS':'25','PreADS':'good','ADSStatus':'good','ADSSoftwareVersion':'CARMA Platform v3.3.1'}"
        //rostopic pub /truck_safety_info std_msgs/String "VIN:2UJGHDV0CLBP8834,CarrierName:FMCSA Tech Division,Inspection:2021-03-30,Calibration:2020-03-30,License:DOT-2003,Weight:12000,Carrier:DOT-2,Permit:No,ISS:25,PreADS:good,ADSStatus:good,ADSSoftwareVersion:CARMA Platform v3.3.1"
        //"VIN:2UJGHDV0CLBP8834,CarrierName:FMCSA Tech Division,Inspection:2021-03-30,Calibration:2020-03-30,License:DOT-2003,Weight:12000,Carrier:DOT-2,Permit:No,ISS:25,PreADS:good,ADSStatus:good,ADSSoftwareVersion:CARMA Platform v3.3.1"
        vin_number:%s,license_plate:%s,carrier_name:%s,carrier_id:%s,weight:%s,ads_software_version:%s,date_of_last_state_inspection:%s,date_of_last_ads_calibration:%s,pre_trip_ads_health_check:%s,ads_status:%s,iss_score:%d,permit_required:%s
        ***/
       
    }



    /***
       * Custom widgets using JQuery Widget Framework.
       * NOTE: that widget framework namespace can only be one level deep.
       * This should not colide with other widgets as loadCustomWidget will be calling private widgets.
       ***/

    $.widget("CarmaJS.truckInspectionSafetyLogList", {
        _create: function () {
            var myDiv = createBtnWithLabelElement(divWidgetOptionsList, 'btnId', "Request Safety Log", "Requested", 'btnName', 'No data',true, 'CarmaJS.WidgetFramework.activateWidget');
            var myDiv1 = createBtnWithLabelElement(divWidgetOptionsList, 'btnId1', "Request Safety Log", "Requested", 'btnName', 'No data',true, 'CarmaJS.WidgetFramework.activateWidget');
            var aceIcon = createImgFramePanelText('aceIconFrameId', 'aceIconFrame', 'taceIconPanelId', 'aceIconPanel', 'aceIconTextId', 'aceIcon', '../../images/ace_icon.png', 'Image is unavailable');      
            var NoMsgAvailable=createDiv("NoMsgAvailableId","NoMsgAvailable","No message availabe");
            
            $(this.element).append(NoMsgAvailable);
            $(this.element).append(myDiv);
            //$(this.element).append(myDiv1);
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

    $.widget("CarmaJS.truckInspectionSafetyLogPerVIN", {
        _create: function () {
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

            $(this.element).append(leftDiv);

            rightDiv.appendChild(truckImage);
            rightDiv.appendChild(logSpan);
            rightDiv.appendChild(aceIcon);

            $(this.element).append(rightDiv);

            console.log("truckInspectionSafetyLogPerVIN element is created");
        },
        _destroy: function () {
            this.element.empty();
            this._super();
            console.log("truckInspectionSafetyLogPerVIN element is destroyed");
        },
        ShowSafetyLogPerVIN: function (messageParam) {
            ShowSafetyLogPerVIN(messageParam);
        }
    });//CarmaJS.truckInspectionSafetyLogPerVIN

    /****Load widget to container. Each widgetName is identical per container and per widget.js */
    var loadCustomWidget = function (container, widgetName) {
        
        //SubscribeToSafetyLog();
        switch (widgetName) {
            case 'list':
                //Safty Log List
                container.truckInspectionSafetyLogList();
                container.truckInspectionSafetyLogList("SafetyLogListNotify", null);
                break;
            case 'detail':
                //Safty Log Detail Per VIN
                container.truckInspectionSafetyLogPerVIN();
                container.truckInspectionSafetyLogPerVIN("ShowSafetyLogPerVIN", null);
                break;
            default:
            //todo
        }
    };

    //*** Public API  ***
    return {
        loadCustomWidget: loadCustomWidget
    };
})();