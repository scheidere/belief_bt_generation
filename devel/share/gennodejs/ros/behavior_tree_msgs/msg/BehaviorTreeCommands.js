// Auto-generated. Do not edit!

// (in-package behavior_tree_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let BehaviorTreeCommand = require('./BehaviorTreeCommand.js');

//-----------------------------------------------------------

class BehaviorTreeCommands {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.commands = null;
    }
    else {
      if (initObj.hasOwnProperty('commands')) {
        this.commands = initObj.commands
      }
      else {
        this.commands = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BehaviorTreeCommands
    // Serialize message field [commands]
    // Serialize the length for message field [commands]
    bufferOffset = _serializer.uint32(obj.commands.length, buffer, bufferOffset);
    obj.commands.forEach((val) => {
      bufferOffset = BehaviorTreeCommand.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BehaviorTreeCommands
    let len;
    let data = new BehaviorTreeCommands(null);
    // Deserialize message field [commands]
    // Deserialize array length for message field [commands]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.commands = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.commands[i] = BehaviorTreeCommand.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.commands.forEach((val) => {
      length += BehaviorTreeCommand.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'behavior_tree_msgs/BehaviorTreeCommands';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6602df19ee620e103a9a9693540b7d4a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    BehaviorTreeCommand[] commands
    ================================================================================
    MSG: behavior_tree_msgs/BehaviorTreeCommand
    string condition_name
    int8 status
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BehaviorTreeCommands(null);
    if (msg.commands !== undefined) {
      resolved.commands = new Array(msg.commands.length);
      for (let i = 0; i < resolved.commands.length; ++i) {
        resolved.commands[i] = BehaviorTreeCommand.Resolve(msg.commands[i]);
      }
    }
    else {
      resolved.commands = []
    }

    return resolved;
    }
};

module.exports = BehaviorTreeCommands;
