// Auto-generated. Do not edit!

// (in-package open_manipulator_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ArmToCar {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.start_time = null;
      this.move = null;
      this.forward_state = null;
    }
    else {
      if (initObj.hasOwnProperty('start_time')) {
        this.start_time = initObj.start_time
      }
      else {
        this.start_time = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('move')) {
        this.move = initObj.move
      }
      else {
        this.move = false;
      }
      if (initObj.hasOwnProperty('forward_state')) {
        this.forward_state = initObj.forward_state
      }
      else {
        this.forward_state = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ArmToCar
    // Serialize message field [start_time]
    bufferOffset = _serializer.time(obj.start_time, buffer, bufferOffset);
    // Serialize message field [move]
    bufferOffset = _serializer.bool(obj.move, buffer, bufferOffset);
    // Serialize message field [forward_state]
    bufferOffset = _serializer.bool(obj.forward_state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ArmToCar
    let len;
    let data = new ArmToCar(null);
    // Deserialize message field [start_time]
    data.start_time = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [move]
    data.move = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [forward_state]
    data.forward_state = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 10;
  }

  static datatype() {
    // Returns string type for a message object
    return 'open_manipulator_msgs/ArmToCar';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4b7661ede9092fffae22ae49adf1666d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    time start_time
    bool move
    bool forward_state
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ArmToCar(null);
    if (msg.start_time !== undefined) {
      resolved.start_time = msg.start_time;
    }
    else {
      resolved.start_time = {secs: 0, nsecs: 0}
    }

    if (msg.move !== undefined) {
      resolved.move = msg.move;
    }
    else {
      resolved.move = false
    }

    if (msg.forward_state !== undefined) {
      resolved.forward_state = msg.forward_state;
    }
    else {
      resolved.forward_state = false
    }

    return resolved;
    }
};

module.exports = ArmToCar;
