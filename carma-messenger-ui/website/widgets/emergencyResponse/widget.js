/***
* Create a unique namespace for each plugin widget to minimize collision of same name variables or functions.
***/
CarmaJS.registerNamespace("CarmaJS.WidgetFramework.emergencyResponse");

CarmaJS.WidgetFramework.emergencyResponse = (function () {
    $.widget("CarmaJS.emergencyResponse", {
        _create: function () {
            let container = document.createElement('div');
            container.className = "emergency-response-container";
            //create title div
            let titleRow = document.createElement('div');
            titleRow.className = "row title-row";
            let titleCol = document.createElement('div');
            titleCol.className = "col";
            let titleImg = "../images/siren.png";
            let img = document.createElement('img');
            img.src = titleImg;
            img.className = "title-img";
            titleCol.appendChild(img);
            let titleLbl = document.createElement('Label');
            titleLbl.innerHTML = "Emergency Response Vehicle Information";
            titleCol.appendChild(titleLbl);
            titleRow.appendChild(titleCol);

            /**
             * create Vehicle status div: License plate, position, velocity, siren status, light status
             */
            let vehicleStatusRow = document.createElement('div');
            vehicleStatusRow.className = "row vehicle-status-row";

            //License plate
            let licenseCol = document.createElement('div');
            licenseCol.className = "col"
            var licenseLabel = document.createElement('Label');
            licenseLabel.innerHTML = "License Plate";
            licenseLabel.className = "license-plate-lbl";
            var licenseValue = document.createElement('Label');
            licenseValue.innerHTML = "######";
            licenseCol.appendChild(licenseLabel);
            licenseCol.appendChild(licenseValue);
            vehicleStatusRow.appendChild(licenseCol);

            //position
            let positionCol = document.createElement('div');
            positionCol.className = "col"
            var positionLabel = document.createElement('Label');
            positionLabel.innerHTML = "Position";
            positionLabel.className = "position-lbl";
            var positionValue = document.createElement('Label');
            positionValue.innerHTML = "######";
            positionCol.appendChild(positionLabel);
            positionCol.appendChild(positionValue);
            vehicleStatusRow.appendChild(positionCol);

            //velocity
            let velocityCol = document.createElement('div');
            velocityCol.className = "col"
            var velocityLabel = document.createElement('Label');
            velocityLabel.innerHTML = "Velocity";
            velocityLabel.className = "velocity-lbl";
            var velocityValue = document.createElement('Label');
            velocityValue.innerHTML = "######";
            velocityCol.appendChild(velocityLabel);
            velocityCol.appendChild(velocityValue);
            vehicleStatusRow.appendChild(velocityCol);

            //siren status
            let sirenCol = document.createElement('div');
            sirenCol.className = "col"
            var sirenLabel = document.createElement('Label');
            sirenLabel.innerHTML = "Siren Status";
            sirenLabel.className = "siren-lbl";
            var sirenValue = document.createElement('Label');
            sirenValue.innerHTML = "######";
            sirenCol.appendChild(sirenLabel);
            sirenCol.appendChild(sirenValue);
            vehicleStatusRow.appendChild(sirenCol);

            //light status
            let lightCol = document.createElement('div');
            lightCol.className = "col";
            var lightLabel = document.createElement('Label');
            lightLabel.innerHTML = "Light Status";
            lightLabel.className = "light-lbl";
            var lightValue = document.createElement('Label');
            lightValue.innerHTML = "######";
            lightCol.appendChild(lightLabel);
            lightCol.appendChild(lightValue);
            vehicleStatusRow.appendChild(lightCol);

            /**
             * Route selction div and arrive at destination button
             */
            let destinationRow = document.createElement('div');
            destinationRow.className = "row destination-row";
            let destinationCol = document.createElement('div');
            destinationCol.className = "col";
            var destLbl = document.createElement('Label');
            destLbl.innerHTML = "Emergency Destination";
            destLbl.className = "dest-lbl";
            destinationCol.appendChild(destLbl);
            let destBtnGrp = document.createElement('div');
            destBtnGrp.className = "btn-group dropright";
            let destDropDownBtn = document.createElement('button');
            destDropDownBtn.className = "btn btn-outline-secondary btn-lg dropdown-toggle";
            destDropDownBtn.id = "dropdownMenuButton";
            destDropDownBtn.innerHTML = "<i class=\"fas fa-route\"></i>&nbsp; Select a route &nbsp;";
            destDropDownBtn.setAttribute("data-toggle", "dropdown");
            destBtnGrp.appendChild(destDropDownBtn);
            let destinationMenu = document.createElement('div');
            destinationMenu.className = "dropdown-menu";
            destinationMenu.setAttribute("aria-labelledby", "dropdownMenuButton");
            let destinationItem = document.createElement('a');
            destinationItem.className = "dropdown-item";
            destinationItem.href = "#";
            destinationItem.innerHTML = "route 1";
            destinationMenu.appendChild(destinationItem);
            destinationCol.appendChild(destinationMenu);
            destBtnGrp.appendChild(destinationMenu);
            destinationCol.appendChild(destBtnGrp);
            let destBtn = document.createElement('button');
            destBtn.className = "btn btn-danger dest-btn btn-lg";
            destBtn.innerHTML = "Arrive at destination";
            destBtn.setAttribute("title", "You will be redirected to event management page.");
            destinationCol.appendChild(destBtn);
            destinationRow.appendChild(destinationCol);

            //Alert popup 
            let alertRow = document.createElement('div');
            alertRow.className = "row alert-row";
            let alertCol = document.createElement('div');
            alertCol.className = "col offset-sm-7";
            let alert = document.createElement('div');
            alert.className = "alert alert-danger alert-dismissible fade show";
            alert.innerHTML = "<strong>This is a sample alert!</strong>";
            let alertCloseBtn = document.createElement('button');
            alertCloseBtn.className = "close";
            alertCloseBtn.setAttribute("data-dismiss", "alert");
            alertCloseBtn.setAttribute("aria-label", "Close");
            let alertSpan = document.createElement('span');
            alertSpan.setAttribute("aria-hidden", "true");
            alertSpan.innerHTML = "&times;";
            alertCloseBtn.appendChild(alertSpan);
            alert.appendChild(alertCloseBtn);
            alertCol.appendChild(alert);
            alertRow.appendChild(alertCol);

            container.appendChild(titleRow);
            container.appendChild(vehicleStatusRow);
            container.appendChild(destinationRow);
            container.appendChild(alertRow);
            $(this.element).append(container);
        },
        subscribe_bsm: function () { },
        service_get_awailable_routes: function () { },
        service_set_route: function () { },
        _destroy: function () {
            this.element.empty();
            this._super();
            console.log("Emergency Response configuration page is destroyed");
        }
    });//CarmaJS.

    var loadCustomWidget = function (container) {
        //create Emergency Response page
        container.emergencyResponse();
        container.emergencyResponse("subscribe_bsm", null);
        container.emergencyResponse("service_get_awailable_routes", null);
        container.emergencyResponse("service_set_route", null);
    };

    //*** Public API  ***
    return {
        loadCustomWidget: loadCustomWidget
    };
})();