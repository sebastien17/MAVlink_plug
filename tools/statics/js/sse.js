//Scheduler
var sched_data = {
    init:[
        stream_init,
        fi_init,
        init_event,
        init_cesiumjs,
        init_waypointsTable
    ],
    refresh:[
        [ fi_refresh,  100],
    ]
}

//Globals
//Cesium js
var cjs = {}
//Flight Indicator
var fi = {}
//Stream
var stream = {}
//Interface
var interf = {}




//Host information
var _hostname = window.location.hostname;
var _port = window.location.port;




$( document ).ready(function() {
    scheduler()
});

/****************************************************************************
*
* Scheduler Logics
*
*****************************************************************************/ 
function scheduler(){
    for(i=0; i<sched_data.init.length;i++){
        sched_data.init[i]();
    }
    for(i=0; i<sched_data.refresh.length;i++){
        setInterval(sched_data.refresh[i][0], sched_data.refresh[i][1]);
    }
}

/****************************************************************************
*
* Server Side Event Logics
*
*****************************************************************************/  
function stream_init(){
    stream.data = {};
    stream.msg_count = 0;
    stream.sse = new EventSource('/stream');
    stream.sse.onmessage = stream_read
}

function stream_read(event) {
    stream.msg_count += 1;
    res = event.data.split(" ",1);
    msg_ident = res[0];
    msg_data = event.data.substring(msg_ident.length+1);
    temp = JSON.parse(msg_data);
    if(!(msg_ident in stream.data)){
        stream.data[msg_ident] = {}
        stream.data[msg_ident].time_boot_ms = 0
        if(interf.current_ident == {}){
            interf.current_ident = msg_ident
        }
    }
    for (data_type in temp){
        if(!(data_type in stream.data[msg_ident])){
            stream.data[msg_ident][data_type] = {}
        }
        if(data_type == 'ATTITUDE'){
            stream.data[msg_ident].time_boot_ms = temp[data_type].time_boot_ms
        }
        stream.data[msg_ident][data_type] = temp[data_type]
        stream.data[msg_ident][data_type].ts = stream.data[msg_ident].time_boot_ms
    }
}

/****************************************************************************
*
* Cesiumjs Logics
*
*****************************************************************************/  
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
    cjs.billboards = cjs.scene.primitives.add(new Cesium.BillboardCollection());
    cjs.waypoints = cjs.scene.primitives.add(new Cesium.BillboardCollection());
    
    //Global Lighting
    cjs.globe.enableLighting = true;
    
    //cjs_update_switch logics
    cjs.map_update = false;
    $("#cjs_uav_location_switch").click(function() {
        cjs.map_update = cjs.map_update ? false : true;
    });
    $("#cjs_fly_to_actual").click(function() {
        navigator.geolocation.getCurrentPosition(cjs.fly);
    });
}

    // Create callback for browser's geolocation
cjs.fly = function(position) {
    cjs.scene.camera.flyTo({
        destination : Cesium.Cartesian3.fromDegrees(position.coords.longitude, position.coords.latitude, 1000.0)
        });
    };


/****************************************************************************
*
* Flight Indicator Logics
*
*****************************************************************************/   
function fi_init(){
     //Initializing FI
    fi.options = {
        size : 150,             // Sets the size in pixels of the indicator (square)
        showBox : false,         // Sets if the outer squared box is visible or not (true or false)
        img_directory : 'statics/vendor/fi/img/'  // The directory where the images are saved to
    }
    fi.attitude = $.flightIndicator('#attitude', 'attitude', fi.options);
    fi.heading = $.flightIndicator('#heading', 'heading', fi.options);
    
    //Setting FI updates

    
    //fi_update_switch logics
    fi.fi_update = false;
    $("#fi_update_switch").click(function() {
        fi.fi_update = fi.fi_update ? false : true;
    });
}
function fi_refresh() {
        if(fi.fi_update){
                    fi.attitude.setRoll((-stream.data['ATTITUDE']['roll']/Math.PI*180).toFixed(3));
                    fi.attitude.setPitch((stream.data['ATTITUDE']['pitch']/Math.PI*180).toFixed(3));
                    fi.heading.setHeading((stream.data['ATTITUDE']['yaw']/Math.PI*180).toFixed(3));
        }
    }

    
/****************************************************************************
*
* Interface Events Logics
*
*****************************************************************************/
function init_event(){

};

/* <PARAM1> <PARAM2> <PARAM3> <PARAM4> <PARAM5/X/LONGITUDE> <PARAM6/Y/LATITUDE> <PARAM7/Z/ALTITUDE> <AUTOCONTINUE>*/
function init_waypointsTable(){  
    $('#waypointsTable').appendGrid({
        initRows: 1,
        columns: [
                { name: 'current_wpt', display: 'Current Wpt', type: 'checkbox' },
                { name: 'c_frame', display: 'C. Frame', type: 'select', ctrlOptions:{  0: 'MAV_FRAME_GLOBAL',
                                                                                        1: 'MAV_FRAME_LOCAL_NED',
                                                                                        2: 'MAV_FRAME_MISSION',
                                                                                        3: 'MAV_FRAME_GLOBAL_RELATIVE_ALT',
                                                                                        4: 'MAV_FRAME_LOCAL_ENU',
                                                                                        7: 'MAV_FRAME_LOCAL_OFFSET_NED',
                                                                                        8: 'MAV_FRAME_BODY_NED',
                                                                                        9: 'MAV_FRAME_BODY_OFFSET_NED',
                                                                                        10: 'MAV_FRAME_GLOBAL_TERRAIN_ALT' 
                                                                                    } 
                },
                { name: 'command', display: 'Command', type: 'select', ctrlOptions: {   16: 'MAV_CMD_NAV_WAYPOINT',
                                                                                        17: 'MAV_CMD_NAV_LOITER_UNLIM',
                                                                                        18: 'MAV_CMD_NAV_LOITER_TURNS',
                                                                                        19: 'MAV_CMD_NAV_LOITER_TIME',
                                                                                        20: 'MAV_CMD_NAV_RETURN_TO_LAUNCH',
                                                                                        21: 'MAV_CMD_NAV_LAND',
                                                                                        22: 'MAV_CMD_NAV_TAKEOFF',
                                                                                        80: 'MAV_CMD_NAV_ROI',
                                                                                        81: 'MAV_CMD_NAV_PATHPLANNING',
                                                                                        82: 'MAV_CMD_NAV_SPLINE_WAYPOINT',
                                                                                        92: 'MAV_CMD_NAV_GUIDED_ENABLE'
                                                                                    } 
                }
                ,
                { name: 'param1', display: 'Par. 1', type: 'text', ctrlAttr: { maxlength: 20 }, ctrlCss: { width: '80px'} },
                { name: 'param2', display: 'Par. 2', type: 'text', ctrlAttr: { maxlength: 20 }, ctrlCss: { width: '80px'} },
                { name: 'param3', display: 'Par. 3', type: 'text', ctrlAttr: { maxlength: 20 }, ctrlCss: { width: '80px'} },
                { name: 'param4', display: 'Par. 4', type: 'text', ctrlAttr: { maxlength: 20 }, ctrlCss: { width: '80px'} },
                { name: 'param5', display: 'Par. 5/x', type: 'text', ctrlAttr: { maxlength: 20 }, ctrlCss: { width: '80px'} },
                { name: 'param6', display: 'Par. 6/y', type: 'text', ctrlAttr: { maxlength: 20 }, ctrlCss: { width: '80px'} },
                { name: 'param7', display: 'Par. 7/z', type: 'text', ctrlAttr: { maxlength: 20 }, ctrlCss: { width: '80px'} },
                { name: 'autocontinue', display: 'Autocontinue', type: 'checkbox' },
                { name: 'RecordId', type: 'hidden', value: 0 }
            ]
    });
}



