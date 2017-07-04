function RosController()
{
    // bind event listeners to button clicks //
    var that = this;

    this.url = 'ws://192.168.1.201:9090';
    //this.url = 'ws://localhost:9090';

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
    /* --------------------------------------------------------- */


    /* Subscribe to calibration state -------------------------- */
    this.imu_1_cal_system = 0;
    this.imu_1_cal_gyroscope = 0;
    this.imu_1_cal_accelerometer = 0;
    this.imu_1_cal_magnetometer = 0;

    this.steer_cal_system = 0;
    this.steer_cal_gyroscope = 0;
    this.steer_cal_accelerometer = 0;
    this.steer_cal_magnetometer = 0;

    this.sub_lean_calibration_state = new ROSLIB.Topic({
        ros : that.ros,
        name : '/bicycle/imu_1_calibration',
        messageType : 'autonomous_bicycle/calibration_status'
    });

    this.sub_lean_calibration_state.subscribe(function(message) {
        that.imu_1_cal_system = message.cal_system;
        that.imu_1_cal_gyroscope = message.cal_gyroscope;
        that.imu_1_cal_accelerometer = message.cal_accelerometer;
        that.imu_1_cal_magnetometer = message.cal_magnetometer;

        that.update_calibration_status()
        // console.log('Received message on ' + that.sub_calibration_state.name + ': ' + message.cal_system);
    });

    this.sub_steer_calibration_state = new ROSLIB.Topic({
        ros : that.ros,
        name : '/bicycle/imu_steer_calibration',
        messageType : 'autonomous_bicycle/calibration_status'
    });

    this.sub_steer_calibration_state.subscribe(function(message) {
        that.steer_cal_system = message.cal_system;
        that.steer_cal_gyroscope = message.cal_gyroscope;
        that.steer_cal_accelerometer = message.cal_accelerometer;
        that.steer_cal_magnetometer = message.cal_magnetometer;

        that.update_calibration_status()
        // console.log('Received message on ' + that.sub_calibration_state.name + ': ' + message.cal_system);
    });

    this.update_calibration_status = function () {
        $("#tb-acc-lean").text(that.imu_1_cal_accelerometer);
        $("#tb-mag-lean").text(that.imu_1_cal_magnetometer);
        $("#tb-gyr-lean").text(that.imu_1_cal_gyroscope);
        $("#tb-sys-lean").text(that.imu_1_cal_system);

        $("#tb-acc-steer").text(that.steer_cal_accelerometer);
        $("#tb-mag-steer").text(that.steer_cal_magnetometer);
        $("#tb-gyr-steer").text(that.steer_cal_gyroscope);
        $("#tb-sys-steer").text(that.steer_cal_system);
    };
    /* --------------------------------------------------------- */

    /* Enable calibration functions ---------------------------- */
    this.service_calibration_lean = new ROSLIB.Service({
        ros : that.ros,
        name : '/bicycle/calibration_imu_1',
        serviceType : 'autonomous_bicycle/calibration_msg'
    });

    this.service_calibration_steer = new ROSLIB.Service({
        ros : that.ros,
        name : '/bicycle/calibration_imu_steering',
        serviceType : 'autonomous_bicycle/calibration_msg'
    });

    this.calibrate_sensors = function (load_calibration, set_offset, reset_imu) {
        var request = new ROSLIB.ServiceRequest({
            load_calibration : load_calibration,
            set_offset : set_offset,
            enable_calibration_status : true,
            reset_imu : reset_imu,
            print_calibration_vector : false
        });

        that.service_calibration_lean.callService(request, function(result) {
            console.log('Calibration Lean: ' + result.result);
        });

        that.service_calibration_steer.callService(request, function(result) {
            console.log('Calibration steering: ' + result.result);
        });
    };

    $("#btn-reset-IMU").on("mouseup", function(event) {
        that.calibrate_sensors(false, false, true)
    });

    $("#btn-load-calibration").on("mouseup", function(event) {
        that.calibrate_sensors(true, false, false)
    });

    $("#btn-set-offset").on("mouseup", function(event) {
        that.calibrate_sensors(false, true, false)
    });
    /* --------------------------------------------------------- */

    /* Record ROSBAG ------------------------------------------- */
    this.filename = 'test_javascript.bag';

    this.service_status = new ROSLIB.Service({
        ros : that.ros,
        name : '/check_bag_record_status',
        serviceType : 'autonomous_bicycle/status_msg'
    });

    this.check_recording_status = function () {
        var request = new ROSLIB.ServiceRequest({
            check_status : true
        });

        that.service_status.callService(request, function(result) {
            console.log('Recording status:' + result.status_record_bag);
            that.is_recording = result.status_record_bag;
            that.update_record_button();
        });
    };

    this.service_control = new ROSLIB.Service({
        ros : that.ros,
        name : '/change_bag_record',
        serviceType : 'autonomous_bicycle/record_msg'
    });

    this.check_recording_status();

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

    this.update_record_button = function () {
        var button = $("#btn-start-record");
        if (that.is_recording){
            button.removeClass('btn-primary').addClass('btn-info');
            button.text('Stop recording');
            that.start_record_bag();
        }
        else{
            button.removeClass('btn-info').addClass('btn-primary');
            button.text('Start recording');
            that.stop_record_bag();
        }
    };

    $('#file_name_input').on('keyup', function(e) {
        var val = $(this).val();
        if (val.match(/[^a-zA-Z]/g)) {
            $(this).val(val.replace(/[^a-zA-Z0-9]/g, ''));
        }
    });
    $("#btn-start-record").on( "mouseup", function( event ) {
        that.is_recording = !that.is_recording;
        that.update_record_button()
    });
    /* ------------------------------------------------------------- */

    /* Turn off service  ------------------------------------------- */
    this.service_turn_off_system = new ROSLIB.Service({
        ros : that.ros,
        name : '/bicycle/turn_off',
        serviceType : 'autonomous_bicycle/turn_off_msg'
    });

    this.turn_off_system = function () {
        var request = new ROSLIB.ServiceRequest({
            turn_off : true
        });

        that.update_turn_off_button(true);
        console.log('Start: service turn off system');
        that.service_turn_off_system.callService(request, function(result) {
            console.log('Completed Turn off system: ' + result.result);
            that.update_turn_off_button(false);
        });
    };

    this.update_turn_off_button = function (is_turning_off) {
        var button = $("#btn-turn-off");
        if (is_turning_off){
            button.removeClass('btn-danger').addClass('btn-warning');
        }
        else{
            button.removeClass('btn-warning').addClass('btn-danger');
        }
    };

    $("#btn-turn-off").on("mouseup", function(event) {
        that.turn_off_system()
    });
    /* ------------------------------------------------------------- */

}
