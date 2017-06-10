function RosController()
{
    // bind event listeners to button clicks //
    var that = this;

    this.url = 'ws://192.168.0.210:9090';

    this.is_recording = false;

    this.imu_1_enabled = 0;
    this.steering_enabled = 0;
    this.camera_enabled = 0;
    this.altitude_enabled = 0;
    this.gps_front_enabled = 0;
    this.gps_rear_enabled = 0;

    // Connecting to ROS.
    this.ros = new ROSLIB.Ros({
        url: this.url
    });

    this.ros.on('connection', function() {
        console.log ('Connected to websocket server.');
    });

    this.ros.on('error', function(error) {
        console.log (error);
    });

    this.ros.on('close', function() {
        console.log('Connection to websocket server closed.');
    });
    /* Subscribe to ROS system state --------------------------- */

    this.sub_system_state = new ROSLIB.Topic({
        ros : that.ros,
        name : '/bicycle/state_system',
        messageType : 'std_msgs/Int8MultiArray'
    });

    this.toggle_indicator = function (enabled, control) {
        $(control).removeClass('btn-warning');
        if (enabled > 0){
            $(control).removeClass('btn-danger').addClass('btn-success');
        }
        else{
            $(control).removeClass('btn-success').addClass('btn-danger');
        }
    };

    this.update_state_indicators = function () {
        that.toggle_indicator(that.imu_1_enabled, "#btn-imu-lean");
        that.toggle_indicator(that.steering_enabled, "#btn-imu-steer");
        that.toggle_indicator(that.camera_enabled, "#btn-camera");
        that.toggle_indicator(that.altitude_enabled, "#btn-altimeter");
        that.toggle_indicator(that.gps_front_enabled, "#btn-gps-front");
        that.toggle_indicator(that.gps_rear_enabled, "#btn-gps-rear");
    };

    this.sub_system_state.subscribe(function(message) {
        that.imu_1_enabled = message.data[0];
        that.steering_enabled = message.data[1];
        that.camera_enabled = message.data[2];
        that.altitude_enabled = message.data[3];
        that.gps_front_enabled = message.data[4];
        that.gps_rear_enabled = message.data[5];

        that.update_state_indicators();

        //console.log('Received message on ' + that.sub_system_state.name + ': ' + message.data);
    });


    /* Record ROSBAG ------------------------------------------- */
    this.filename = 'test_javascript.bag';

    this.service_control = new ROSLIB.Service({
        ros : that.ros,
        name : '/change_bag_record',
        serviceType : 'autonomous_bicycle/record_msg'
    });

    this.start_record_bag = function () {
        console.log ('Start recording ROSBAG');

        // File name based on user input
        that.filename = $('#file_name_input').val() + '.bag';

        var request = new ROSLIB.ServiceRequest({
            filename : that.filename,
            enable_record : true
        });

        that.service_control.callService(request, function(result) {
            console.log('Result:' + result.result);
        });
    };

    this.stop_record_bag = function () {
        var request = new ROSLIB.ServiceRequest({
            filename : that.filename,
            enable_record : false
        });

        this.service_control.callService(request, function(result) {
            console.log('Result:' + result.result);
        });
    };

    /* ------------------------------------------------------------- */
    $("#btn-start-record").on( "mouseup", function( event ) {
        that.is_recording = !that.is_recording;

        if (that.is_recording){
            $(this).removeClass('btn-primary').addClass('btn-info');
            $(this).text('Stop recording');
            that.start_record_bag();
        }
        else{
            $(this).removeClass('btn-info').addClass('btn-primary');
            $(this).text('Start recording');
            that.stop_record_bag();
        }
    });

    $('#file_name_input').on('keyup', function(e) {
        var val = $(this).val();
        if (val.match(/[^a-zA-Z]/g)) {
            $(this).val(val.replace(/[^a-zA-Z0-9]/g, ''));
        }
    });
}
