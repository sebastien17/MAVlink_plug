//Global definition 

//Cesium js
var cjs_viewer;
var cjs_scene;
var cjs_globe;
var cjs_billboards;
var cjs_waypoints;
var cjs_uav; 
var cjs_map_update = false;
var UAV_waypoints;

//Host information
var _hostname = window.location.hostname;
var _port = window.location.port;

//Flight Indicator
var fi_update = false;

//Data
var global_data = {};
var msg_count = 0;

$( document ).ready(function() {
    init_stream()
    init_fi();
    //init_event();
    //init_cesiumjs();
    //init_waypointsTable();
});

function init_stream(){
    
    sse = new EventSource('/stream');
    sse.onmessage = function(event) {
        msg_count += 1;
        res = event.data.split(" ",1);
        msg_type = res[0];
        msg_data = event.data.substring(msg_type.length+1);
        temp = JSON.parse(msg_data);
        global_data[msg_type] = temp;
    }
}



function init_cesiumjs(){
    
    var ajax_port = _port;
    var ajax_hostname = _hostname;
    
    //Init cesiumjs viewer
    cjs_viewer = new Cesium.Viewer('cesiumContainer',{  timeline : false,
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
    cjs_scene = cjs_viewer.scene;
    cjs_globe = cjs_scene.globe;
    cjs_billboards = cjs_scene.primitives.add(new Cesium.BillboardCollection());
    cjs_waypoints = cjs_scene.primitives.add(new Cesium.BillboardCollection());
    
    //Global Lighting
    cjs_globe.enableLighting = true;
    
    // Ask browser for location, and fly there.
    navigator.geolocation.getCurrentPosition(fly);
    
    // Create callback for browser's geolocation
    function fly(position) {
    cjs_scene.camera.flyTo({
        destination : Cesium.Cartesian3.fromDegrees(position.coords.longitude, position.coords.latitude, 1000.0)
        });
    };
    //Add icon to billboard
        cjs_uav = cjs_billboards.add({
        horizontalOrigin : Cesium.HorizontalOrigin.CENTER,
        verticalOrigin  :  Cesium.VerticalOrigin.CENTER,
        image : 'img/red_point.png',
        scale : 0.3,
        show : false
    });
        
        /**************************************************************
        billboards.add({
        image : '../images/Cesium_Logo_overlay.png', // default: undefined
        show : true, // default
        position : Cesium.Cartesian3.fromDegrees(-75.59777, 40.03883),
        pixelOffset : new Cesium.Cartesian2(0, -50), // default: (0, 0)
        eyeOffset : new Cesium.Cartesian3(0.0, 0.0, 0.0), // default
        horizontalOrigin : Cesium.HorizontalOrigin.CENTER, // default
        verticalOrigin : Cesium.VerticalOrigin.BOTTOM, // default: CENTER
        scale : 2.0, // default: 1.0
        color : Cesium.Color.LIME, // default: WHITE
        rotation : Cesium.Math.PI_OVER_FOUR, // default: 0.0
        alignedAxis : Cesium.Cartesian3.ZERO, // default
        width : 100, // default: undefined
        height : 25 // default: undefined
    });
    ************************************************************************/


    
    //cjs_update_switch logics
    $("#cjs_update_switch").click(function() {
        cjs_map_update = cjs_map_update ? false : true;
    });
}
    
function init_fi(){
     //Initializing FI
    var options = {
        size : 150,             // Sets the size in pixels of the indicator (square)
        showBox : false,         // Sets if the outer squared box is visible or not (true or false)
        img_directory : 'statics/vendor/fi/img/'  // The directory where the images are saved to
    }
    var attitude = $.flightIndicator('#attitude', 'attitude', options);
    var heading = $.flightIndicator('#heading', 'heading', options);
    
    //Setting FI updates
    setInterval(function() {
        if(fi_update){
                    attitude.setRoll((-global_data['ATTITUDE']['roll']/Math.PI*180).toFixed(3));
                    attitude.setPitch((global_data['ATTITUDE']['pitch']/Math.PI*180).toFixed(3));
                    heading.setHeading((global_data['ATTITUDE']['yaw']/Math.PI*180).toFixed(3));
        }
    }, 100);
    
    //fi_update_switch logics
    $("#fi_update_switch").click(function() {
        fi_update = fi_update ? false : true;
    });
}

    
function init_event(){
    var ajax_port = _port;
    var ajax_hostname = _hostname;
    //Logic behind button with class MAVlink_CMD_button
    $(".MAVlink_CMD_button").click(function() {
            var $this = $( this );
            $.ajax({
                url : ajax_hostname + ajax_port,  
                type : 'POST', 
                data : {topic: "MAVLINK_CMD", type: $this.attr('cmd')},
                dataType : 'json',
                success : function(data, status){
                }
            });
    });
    $("#MAVlink_CMD_WP_request_list_button").click(function() {
            var $this = $( this );
            $.ajax({
                url : ajax_hostname + ajax_port,  
                type : 'POST', 
                data : {topic: "MAVLINK_CMD", type: "WP_LIST_REQUEST"},
                dataType : 'json',
                success : function(data, status){
                    UAV_waypoints = data;
                    update_waypoint();
                }
            });
    });

};

function init_waypointsTable(){  
    $('#waypointsTable').appendGrid({
        initRows: 1,
        columns: [
                { name: 'Album', display: 'Album', type: 'text', ctrlAttr: { maxlength: 100 }, ctrlCss: { width: '160px'} },
                { name: 'Artist', display: 'Artist', type: 'text', ctrlAttr: { maxlength: 100 }, ctrlCss: { width: '100px'} },
                { name: 'Year', display: 'Year', type: 'text', ctrlAttr: { maxlength: 4 }, ctrlCss: { width: '40px'} },
                { name: 'Origin', display: 'Origin', type: 'select', ctrlOptions: { 0: '{Choose}', 1: 'Hong Kong', 2: 'Taiwan', 3: 'Japan', 4: 'Korea', 5: 'US', 6: 'Others'} },
                { name: 'Poster', display: 'With Poster?', type: 'checkbox' },
                { name: 'Price', display: 'Price', type: 'text', ctrlAttr: { maxlength: 10 }, ctrlCss: { width: '50px', 'text-align': 'right' }, value: 0 },
                { name: 'RecordId', type: 'hidden', value: 0 }
            ]
    });
}


function update_waypoint(){
    
    var waypoint_show = {
            image : 'img/black_losange.png',
            horizontalOrigin : Cesium.HorizontalOrigin.CENTER,
            verticalOrigin  :  Cesium.VerticalOrigin.CENTER,
            color : new Cesium.Color(1.0, 1.0, 1.0, 0.7),
            scale : 0.15,
            show : false
        };
    cjs_waypoints.removeAll();    
    for (var key in UAV_waypoints) {
        temp = cjs_waypoints.add(waypoint_show);
        temp.position = Cesium.Cartesian3.fromDegrees(UAV_waypoints[key].y, UAV_waypoints[key].x, UAV_waypoints[key].z);
        temp.show = true;
    }
}