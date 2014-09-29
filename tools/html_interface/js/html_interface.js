$( document ).ready(function() {
    init_fi();
    init_event();
    init_cesiumjs();    
});

function init_cesiumjs(){
    //Init cesiumjs viewer
    var viewer = new Cesium.Viewer('cesiumContainer');
    var scene = viewer.scene;
    // Ask browser for location, and fly there.
    navigator.geolocation.getCurrentPosition(fly);
    
    // Create callback for browser's geolocation
    function fly(position) {
    scene.camera.flyTo({
        destination : Cesium.Cartesian3.fromDegrees(position.coords.longitude, position.coords.latitude, 1000.0)
        });
    };
}
    
function init_fi(){
    var ajax_port = 43017;
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
        $.ajax({
            url : 'http://127.0.0.1:'+ ajax_port, 
            type : 'POST', 
            data : { topic: "ATTITUDE", type: "GET_VALUE"},
            dataType : 'json',
            success : function(data, status){
                attitude.setRoll(-data['roll']/Math.PI*180);
                attitude.setPitch(data['pitch']/Math.PI*180);
                heading.setHeading(data['yaw']/Math.PI*180);
            }
        });
    }, 50);
}

    
function init_event(){
    var ajax_port = 43017;
    $("#reset_button").click(function() {

            $.ajax({
                url : 'http://127.0.0.1:'+ ajax_port, 
                type : 'POST', 
                data : { topic: "RESET", type: "MAVLINK_CMD"},
                dataType : 'json',
                success : function(data, status){
                }
            });
    });
    $("#loiter_button").click(function() {

            $.ajax({
                url : 'http://127.0.0.1:'+ ajax_port, 
                type : 'POST', 
                data : { topic: "LOITER_MODE", type: "MAVLINK_CMD"},
                dataType : 'json',
                success : function(data, status){
                }
            });
    });
};