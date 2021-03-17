// Auto-generated. Do not edit!

// (in-package behavior_tree_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class BehaviorTreeCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.condition_name = null;
      this.status = null;
    }
    else {
      if (initObj.hasOwnProperty('condition_name')) {
        this.condition_name = initObj.condition_name
      }
      else {
        this.condition_name = '';
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BehaviorTreeCommand
    // Serialize message field [condition_name]
    bufferOffset = _serializer.string(obj.condition_name, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = _serializer.int8(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BehaviorTreeCommand
    let len;
    let data = new BehaviorTreeCommand(null);
    // Deserialize message field [condition_name]
    data.condition_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.condition_name.length;
    return length + 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'behavior_tree_msgs/BehaviorTreeCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '88f8877408328a1537655cc1377c588d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string condition_name
    int8 status
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BehaviorTreeCommand(null);
    if (msg.condition_name !== undefined) {
      resolved.condition_name = msg.condition_name;
    }
    else {
      resolved.condition_name = ''
    }

    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = 0
    }

    return resolved;
    }
};

module.exports = BehaviorTreeCommand;
