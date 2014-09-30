//Global definition 

//Cesium js
var cjs_viewer;
var cjs_scene;
var cjs_globe;
var cjs_billboards;
var cjs_uav; 
var cjs_map_update = true;

//Host information
var _hostname = window.location.hostname;
var _port = window.location.port;

//Flight Indicator
var fi_update = true;

$( document ).ready(function() {
    init_fi();
    init_event();
    init_cesiumjs();    
});

function init_cesiumjs(){
    
    var ajax_port = _port;
    var ajax_hostname = _hostname;
    //Init cesiumjs viewer
    cjs_viewer = new Cesium.Viewer('cesiumContainer');
    cjs_scene = cjs_viewer.scene;
    cjs_globe = cjs_scene.globe;
    cjs_billboards = cjs_scene.primitives.add(new Cesium.BillboardCollection());
    
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
        image : 'https://cdn1.iconfinder.com/data/icons/future/512/drones_watching-128.png',
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

    setInterval(function() {
        if(cjs_map_update){
            $.ajax({
                url : ajax_hostname + ajax_port, 
                type : 'POST', 
                data : { topic: "GET_VALUE", type: "GPS_RAW_INT"},
                dataType : 'json',
                success : function(data, status){
                    if(data != {}){
                        lon = data['lon']/10000000;
                        lat = data['lat']/10000000;
                        alt = data['alt']/1000;
                        cjs_uav.position = Cesium.Cartesian3.fromDegrees(lon, lat, alt);
                        cjs_uav.show = true;
                    }
                    else
                    {
                        cjs_uav.show = false
                    }
                }
            });
        };
    }, 1000);
}
    
function init_fi(){
    var ajax_port = _port;
    var ajax_hostname = _hostname;
    var options = {
        size : 200,             // Sets the size in pixels of the indicator (square)
        showBox : false,         // Sets if the outer squared box is visible or not (true or false)
        img_directory : 'fi/img/'  // The directory where the images are saved to
    }
    //Initializing FI 
    var attitude = $.flightIndicator('#attitude', 'attitude', options);
    var heading = $.flightIndicator('#heading', 'heading', options);
    //Setting FI updates
    setInterval(function() {
        if(fi_update){
            $.ajax({
                url : ajax_hostname + ajax_port, 
                type : 'POST', 
                data : { topic: "GET_VALUE", type: "ATTITUDE"},
                dataType : 'json',
                success : function(data, status){
                    attitude.setRoll((-data['roll']/Math.PI*180).toFixed(3));
                    attitude.setPitch((data['pitch']/Math.PI*180).toFixed(3));
                    heading.setHeading((data['yaw']/Math.PI*180).toFixed(3));
                }
            });
        };
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

};