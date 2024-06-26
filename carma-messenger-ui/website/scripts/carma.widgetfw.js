CarmaJS.registerNamespace("CarmaJS.WidgetFramework");

CarmaJS.WidgetFramework = (function () {
          var scriptLoader = function (scripts, callback) {

            //In our scriptLoadHandler function we make sure that jQuery can be used with other libraries and also with other versions of itself.
           //jQuery = window.jQuery.noConflict(true);

            var count = scripts.length;

            function urlCallback(url) {
                return function () {
                    --count;
                    if (count < 1) {
                        callback();
                    }
                };
            }

            function loadScript(url) {

                var s = document.createElement('script');
                s.setAttribute('src', url);
                s.onload = urlCallback(url);
                document.head.appendChild(s);
            }

            for (var script of scripts) {
                loadScript(script);
            }
        };

        var loadEventManagementWidgets = function(){
                var cssFilePath = 'widgets/eventManagement/widget.css';
                var jsFilePath = 'widgets/eventManagement/widget.js';
                $.ajax({
                     url: jsFilePath,
                     type:'HEAD',
                     error: function()
                     {
                         //file not exists
                         //TODO: In chrome, even with statusCode or error handling, the HTTP 404 (Failed to Load) error still shows separately
                         console.log('loadEventManagementWidgets: Widget file does NOT exist: ' + jsFilePath );
                         return false;
                     },
                     success: function()
                     {
                        //console.log('cssFilePath: ' + cssFilePath);
                        //1) Load css
                        var link = document.createElement('link');
                        link.setAttribute('rel', 'stylesheet');
                        link.setAttribute('type', 'text/css');
                        link.setAttribute('href', cssFilePath);
                        document.getElementsByTagName('head')[0].appendChild(link);

                        //2) Load JS
                        //console.log('jsFilePath1: ' + jsFilePath);
                        scriptLoader([jsFilePath],function()
                        {
                             // now you can use the code from loaded script files.
                            eval('CarmaJS.WidgetFramework.eventManagement' + '.loadCustomWidget($("#divWidgetAreaEventManagement"));');
                        });

                        return true;
                     }
                });
        };

        var closeEventManagementWidgets = function () {
            $('#divWidgetAreaEventManagement').empty();
            console.log("closeEventManagementWidgets is called");
        };

        var loadEmergencyResponseWidgets = function(){
            var cssFilePath = 'widgets/emergencyResponse/widget.css';
            var jsFilePath = 'widgets/emergencyResponse/widget.js';
            $.ajax({
                 url: jsFilePath,
                 type:'HEAD',
                 error: function()
                 {
                     //file not exists
                     console.log('loadEmergencyResponsetWidgets: Widget file does NOT exist: ' + jsFilePath );
                     return false;
                 },
                 success: function()
                 {
                    //1) Load css
                    var link = document.createElement('link');
                    link.setAttribute('rel', 'stylesheet');
                    link.setAttribute('type', 'text/css');
                    link.setAttribute('href', cssFilePath);
                    document.getElementsByTagName('head')[0].appendChild(link);

                    //2) Load JS
                    scriptLoader([jsFilePath],function()
                    {
                        // now you can use the code from loaded script files.
                        eval('CarmaJS.WidgetFramework.emergencyResponse' + '.loadCustomWidget($("#divWidgetAreaEmergencyResponse"));');
                    });
                    return true;
                 }
            });
        };

        var closeEmergencyResponseWidgets = function () {
            $('#divWidgetAreaEmergencyResponse').empty();
            console.log("closeEmergencyResponseWidgets is called");
        };
    
        //Public API
        return {
            loadEventManagementWidgets: loadEventManagementWidgets,
            loadEmergencyResponseWidgets: loadEmergencyResponseWidgets,
            closeEventManagementWidgets: closeEventManagementWidgets,
            closeEmergencyResponseWidgets: closeEmergencyResponseWidgets
        };
})();
