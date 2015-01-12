//Globals
//Cesium js
var cjs = {};

//Host information
var _hostname = window.location.hostname;
var _port = window.location.port;

//Scheduler
var sched_data = {
    init:[
        init_event,
        init_cesiumjs
    ],
    refresh:[
    ]
}






$( document ).ready(function() {
    scheduler()
});

/****************************************************************************
*
* Scheduler Logics
*
*****************************************************************************/ 
function scheduler(){
    //Init functions
    for(i=0; i<sched_data.init.length;i++){
        sched_data.init[i]();
    }
    //Cron functions
    for(i=0; i<sched_data.refresh.length;i++){
        setInterval(sched_data.refresh[i][0], sched_data.refresh[i][1]);
    }
}

/****************************************************************************
*
* Cesiumjs Logics
*
*****************************************************************************/  
// Create callback for browser's geolocation
cjs.fly = function(position) {
    cjs.scene.camera.flyTo({
        destination : Cesium.Cartesian3.fromDegrees(position.coords.longitude, position.coords.latitude, 1000.0)
        });
    };

function init_cesiumjs(){
    
    var ajax_port = _port;
    var ajax_hostname = _hostname;
    
    //Init cesiumjs viewer
    cjs.viewer = new Cesium.Viewer('cesiumContainer',{  
        timeline : false,
        homeButton: false, 
        animation: false, 
        navigationHelpButton : false,
        baseLayerPicker : false,
        imageryProvider : new Cesium.BingMapsImageryProvider({
            url : '//dev.virtualearth.net',
            //key : 'get-yours-at-https://www.bingmapsportal.com/',
            mapStyle : Cesium.BingMapsStyle.AERIAL_WITH_LABELS
        })
    });
    cjs.scene = cjs.viewer.scene;
    cjs.globe = cjs.scene.globe;

    
    //Global Lighting
    cjs.globe.enableLighting = true;
    
    // Ask browser for location, and fly there.
    navigator.geolocation.getCurrentPosition(cjs.fly);
    
}


    
/****************************************************************************
*
* Interface Events Logics
*
*****************************************************************************/
function init_event(){

};
