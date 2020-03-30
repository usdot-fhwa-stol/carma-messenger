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
    var SaftyLogListNotify = function () {

        listenerSaftyLogList = new ROSLIB.Topic({
            ros: ros,
            name: '/system_alert',//driver_discovery,
            messageType: 'cav_msgs/SystemAlert'// 'cav_msgs/DriverStatus'
        });

        listenerSaftyLogList.subscribe(function (message) {
            console.log(message.description);
            console.log(message.type);
            if (message.type != null && message.type != 'undefined')
                document.getElementById('lblbtnId').innerHTML = message.description; //TODO: VIN number
            document.getElementById('btnbtnId').name = message.type;// TODO: VIN number
        });
    };
    var ShowSaftyLogPerVIN = function () {

        listenerSaftyLogPerVIN = new ROSLIB.Topic({
            ros: ros,
            name: '/system_alert',//driver_discovery,
            messageType: 'cav_msgs/SystemAlert'// 'cav_msgs/DriverStatus'
        });

        listenerSaftyLogPerVIN.subscribe(function (message) {
            console.log(message.description);
            console.log(message.type);
            if (message.type != null && message.type != 'undefined') {
                document.getElementById('VINTextId').innerHTML = "VIN Yex"//message.description; //TODO: VIN number
                document.getElementById('CarrierNameTextId').innerHTML = "CARMA Platform v#.#.#";// TODO: VIN number
            }
        });

        document.getElementById('VINTextId').innerHTML = "1FUJGHDV0CLBP8834"//message.description; //TODO: VIN number
        document.getElementById('CarrierNameTextId').innerHTML = "FMCSA Tech Division";// TODO: VIN number
        document.getElementById('InspectionTextId').innerHTML = "2020-01-01";// TODO: VIN number
        document.getElementById('CalibrationTextId').innerHTML = "2020-01-02";// TODO: VIN number
        document.getElementById('LicenseTextId').innerHTML = "DOT-10003";// TODO: VIN number
        document.getElementById('WeightTextId').innerHTML = "100,000";// TODO: VIN number
        document.getElementById('CarrierTextId').innerHTML = "DOT 1";// TODO: VIN number
        document.getElementById('PermitTextId').innerHTML = "No";// TODO: VIN number
        document.getElementById('ISSTextId').innerHTML = "23";// TODO: VIN number
        //document.getElementById('PreADSHealthCheckTextId').innerHTML = "23";// TODO: VIN number
        // document.getElementById('ADSHealthStatusTextId').innerHTML = "23";// TODO: VIN number
        document.getElementById('ADSSoftwareVersionTextId').innerHTML = "CARMA Platform v3.3.0";// TODO: VIN number
    };


    /***
       * Custom widgets using JQuery Widget Framework.
       * NOTE: that widget framework namespace can only be one level deep.
       * This should not colide with other widgets as loadCustomWidget will be calling private widgets.
       ***/

    $.widget("CarmaJS.truckInspectionSaftyLogList", {
        _create: function () {
            var myDiv = createBtnWithLabelElement(divWidgetOptionsList, 'btnId', "Request Safety Log", "Requested", 'btnName', 'labelTitle', 'CarmaJS.WidgetFramework.activateWidget');
            var myDiv1 = createBtnWithLabelElement(divWidgetOptionsList, 'btnId1', "Request Safety Log", "Requested", 'btnName', 'labelTitle', 'CarmaJS.WidgetFramework.activateWidget');
            var aceIcon = createImgFramePanelText('aceIconFrameId', 'aceIconFrame', 'taceIconPanelId', 'aceIconPanel', 'aceIconTextId', 'aceIcon', '../../images/ace_icon.png', 'Image is unavailable');

            $(this.element).append(myDiv);
            $(this.element).append(myDiv1);
            $(this.element).append(aceIcon);
            console.log("truckInspectionSaftyLogList element is created");
        },
        _destroy: function () {
            this.element.empty();
            this._super();
            console.log("truckInspectionSaftyLogList element is destroyed");
        },
        SaftyLogListNotify: function () {
            SaftyLogListNotify();
        }
    });//CarmaJS.truckInspectionSaftyLogList

    $.widget("CarmaJS.truckInspectionSaftyLogPerVIN", {
        _create: function () {
            var VINFrameStr = createDivFramePanelText('VINFrameId', 'VINFrame', 'VINPanelId', 'VINPanel', 'VIN', 'VINTextId', 'VINText');
            var CarrierNameFrame = createDivFramePanelText('CarrierNameFrameId', 'CarrierNameFrame', 'CarrierNamePanelId', 'CarrierNamePanel', 'Carrier Name', 'CarrierNameTextId', 'CarrierNameText');
            var InspectionFrame = createDivFramePanelText('InspectionFrameId', 'InspectionFrame', 'InspectionPanelId', 'InspectionPanel', 'Last State Inspection', 'InspectionTextId', 'InspectionText');
            var CalibrationFrame = createDivFramePanelText('CalibrationFrameId', 'CalibrationFrame', 'CalibrationPanelId', 'CalibrationPanel', 'Last ADS Calibration', 'CalibrationTextId', 'CalibrationText');
            var LicenseFrame = createDivFramePanelText('LicenseFrameId', 'LicenseFrame', 'LicensePanelId', 'LicensePanel', 'License', 'LicenseTextId', 'LicenseText');
            var WeightFrame = createDivFramePanelText('WeightFrameId', 'WeightFrame', 'WeightPanelId', 'WeightPanel', 'Weight(lbs.)', 'WeightTextId', 'WeightText');
            var CarrierFrame = createDivFramePanelText('CarrierFrameId', 'CarrierFrame', 'CarrierPanelId', 'CarrierPanel', 'Carrier ID', 'CarrierTextId', 'CarrierText');
            var PermitFrame = createDivFramePanelText('PermitFrameId', 'PermitFrame', 'PermitPanelId', 'PermitPanel', 'Permit Required', 'PermitTextId', 'PermitText');
            var ISSFrame = createDivFramePanelText('ISSFrameId', 'ISSFrame', 'ISSPanelId', 'ISSPanel', 'ISS', 'ISSTextId', 'ISSText');
            var PreADSHealthCheckFrame = createDivFramePanelText('PreADSHealthCheckFrameId', 'PreADSHealthCheckFrame', 'PreADSHealthCheckPanelId', 'PreADSHealthCheckPanel', 'ADS Pre-Trip Check', 'PreADSHealthCheckTextId', 'PreADSHealthCheckText');
            var ADSHealthStatusFrame = createDivFramePanelText('ADSHealthStatusFrameId', 'ADSHealthStatusFrame', 'ADSHealthStatusPanelId', 'ADSHealthStatusPanel', 'ADS Health Status', 'ADSHealthStatusTextId', 'ADSHealthStatusText');
            var ADSSoftwareVersionFrame = createDivFramePanelText('ADSSoftwareVersionFrameId', 'ADSSoftwareVersionFrame', 'ADSSoftwareVersionPanelId', 'ADSSoftwareVersionPanel', 'ADS Software Version', 'ADSSoftwareVersionTextId', 'ADSSoftwareVersionText');
            var truckImage = createImgFramePanelText('truckImgFrameId', 'truckImgFrame', 'truckImgPanelId', 'truckImgPanel', 'truckImgTextId', 'truckImg', '../../images/truck_1FUJGHDV0CLBP8834.png', 'Image is unavailable');
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

            console.log("truckInspectionSaftyLogPerVIN element is created");
        },
        _destroy: function () {
            this.element.empty();
            this._super();
            console.log("truckInspectionSaftyLogPerVIN element is destroyed");
        },
        ShowSaftyLogPerVIN: function () {
            ShowSaftyLogPerVIN();
        }
    });//CarmaJS.truckInspectionSaftyLogPerVIN

    /****Load widget to container. Each widgetName is identical per container and per widget.js */
    var loadCustomWidget = function (container, widgetName) {
        switch (widgetName) {
            case 'list':
                //Safty Log List
                container.truckInspectionSaftyLogList();
                container.truckInspectionSaftyLogList("SaftyLogListNotify", null);
                break;
            case 'detail':
                //Safty Log Detail Per VIN
                container.truckInspectionSaftyLogPerVIN();
                container.truckInspectionSaftyLogPerVIN("ShowSaftyLogPerVIN", null);
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