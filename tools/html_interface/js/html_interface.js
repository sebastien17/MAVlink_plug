$( document ).ready(function() {
    init_fi();
    var viewer = new Cesium.Viewer('cesiumContainer');
});
    
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
                data : { topic: "ATTITUDE"},
                dataType : 'json',
                success : function(data, status){
                    attitude.setRoll(-data['roll']/Math.PI*180);
                    attitude.setPitch(data['pitch']/Math.PI*180);
                    heading.setHeading(data['yaw']/Math.PI*180);
                }
            });
        }, 50);
    }
