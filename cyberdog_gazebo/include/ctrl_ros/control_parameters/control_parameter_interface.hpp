/*! @file control_parameter_interface.hpp
 *  @brief Types to allow remote access of control parameters, for use with
 * LCM/Shared memory
 *
 * There are response/request messages.  The robot receives request messages and
 * responds with response messages The request messages either request setting a
 * parameter or getting a parameter
 */

#ifndef PROJECT_CONTROLPARAMETERINTERFACE_H
#define PROJECT_CONTROLPARAMETERINTERFACE_H

#include "control_parameters/control_parameters.hpp"
#include <map>

/*!
 * Type of message to a control parameter collection
 */
enum class ControlParameterRequestKind {
    kGET_ROBOT_PARAM_BY_NAME,
    kSET_ROBOT_PARAM_BY_NAME,
    kGET_USER_PARAM_BY_NAME,
    kSET_USER_PARAM_BY_NAME,
    kGET_SPEED_CALIBRATE_PARAM_BY_NAME,
    kSET_SPEED_CALIBRATE_PARAM_BY_NAME,
    kGET_IMU_CALIBRATE_PARAM_BY_NAME,
    kSET_IMU_CALIBRATE_PARAM_BY_NAME
};

std::string ControlParameterRequestKindToString( ControlParameterRequestKind request );

/*!
 * Data sent to a control parameter collection to request a get/set of a value
 */
struct ControlParameterRequest {
    char                        name[ kCONTROL_PARAMETER_MAXIMUM_NAME_LENGTH ] = "";  // name of the parameter to set/get
    u64                         requestNumber                                 = UINT64_MAX;
    ControlParameterValue       value;
    ControlParameterValueKind   parameterKind;
    ControlParameterRequestKind requestKind;

    /*!
     * Convert to human-readable string
     * @return : description of the request
     */
    std::string ToString() {
        std::string result = "Request(" + std::to_string( requestNumber ) + ") " + ControlParameterRequestKindToString( requestKind ) + " " + ControlParameterValueKindToString( parameterKind ) + " "
                             + std::string( name ) + " ";
        switch ( requestKind ) {
        case ControlParameterRequestKind::kGET_USER_PARAM_BY_NAME:
            result += "user is: ";
            result += ControlParameterValueToString( value, parameterKind );
            return result;
        case ControlParameterRequestKind::kSET_USER_PARAM_BY_NAME:
            result += "user to: ";
            result += ControlParameterValueToString( value, parameterKind );
            return result;
        case ControlParameterRequestKind::kGET_ROBOT_PARAM_BY_NAME:
            result += "robot is: ";
            result += ControlParameterValueToString( value, parameterKind );
            return result;
        case ControlParameterRequestKind::kSET_ROBOT_PARAM_BY_NAME:
            result += "robot to: ";
            result += ControlParameterValueToString( value, parameterKind );
            return result;
        case ControlParameterRequestKind::kGET_SPEED_CALIBRATE_PARAM_BY_NAME:
            result += "speed_calibrate is: ";
            result += ControlParameterValueToString( value, parameterKind );
            return result;
        case ControlParameterRequestKind::kSET_SPEED_CALIBRATE_PARAM_BY_NAME:
            result += "speed_calibrate to: ";
            result += ControlParameterValueToString( value, parameterKind );
            return result;
        case ControlParameterRequestKind::kGET_IMU_CALIBRATE_PARAM_BY_NAME:
            result += "imu_calibrate is: ";
            result += ControlParameterValueToString( value, parameterKind );
            return result;
        case ControlParameterRequestKind::kSET_IMU_CALIBRATE_PARAM_BY_NAME:
            result += "imu_calibrate to: ";
            result += ControlParameterValueToString( value, parameterKind );
            return result;
        default:
            return result + " unknown request type!";
        }
    }
};

/*!
 * Data sent from a control parameter collection in response to a request.
 */
struct ControlParameterResponse {
    char                        name[ kCONTROL_PARAMETER_MAXIMUM_NAME_LENGTH ] = "";
    u64                         requestNumber                                 = UINT64_MAX;
    u64                         nParameters                                   = 0;
    ControlParameterValue       value;
    ControlParameterValueKind   parameterKind;
    ControlParameterRequestKind requestKind;

    /*!
     * Check if a response is a valid response to a given request
     * @param request : the request
     * @return is the response from the given request?
     */
    bool isResponseTo( ControlParameterRequest& request ) {
        return requestNumber == request.requestNumber && requestKind == request.requestKind && std::string( name ) == std::string( request.name );
    }

    /*!
     * Convert to human-readable string
     * @return : description of the response
     */
    std::string ToString() {
        std::string result = "Response(" + std::to_string( requestNumber ) + ") " + ControlParameterRequestKindToString( requestKind ) + " " + ControlParameterValueKindToString( parameterKind ) + " "
                             + std::string( name ) + " ";

        switch ( requestKind ) {
        case ControlParameterRequestKind::kSET_USER_PARAM_BY_NAME:
            result += "user to: ";
            result += ControlParameterValueToString( value, parameterKind );
            return result;
        case ControlParameterRequestKind::kGET_USER_PARAM_BY_NAME:
            result += "user is: ";
            result += ControlParameterValueToString( value, parameterKind );
            return result;
        case ControlParameterRequestKind::kSET_ROBOT_PARAM_BY_NAME:
            result += "robot to: ";
            result += ControlParameterValueToString( value, parameterKind );
            return result;
        case ControlParameterRequestKind::kGET_ROBOT_PARAM_BY_NAME:
            result += "robot is: ";
            result += ControlParameterValueToString( value, parameterKind );
            return result;
        case ControlParameterRequestKind::kGET_SPEED_CALIBRATE_PARAM_BY_NAME:
            result += "speed_calibrate is: ";
            result += ControlParameterValueToString( value, parameterKind );
            return result;
        case ControlParameterRequestKind::kSET_SPEED_CALIBRATE_PARAM_BY_NAME:
            result += "speed_calibrate to: ";
            result += ControlParameterValueToString( value, parameterKind );
            return result;
        case ControlParameterRequestKind::kGET_IMU_CALIBRATE_PARAM_BY_NAME:
            result += "imu_calibrate is: ";
            result += ControlParameterValueToString( value, parameterKind );
            return result;
        case ControlParameterRequestKind::kSET_IMU_CALIBRATE_PARAM_BY_NAME:
            result += "imu_calibrate to: ";
            result += ControlParameterValueToString( value, parameterKind );
            return result;
        default:
            return result + " unknown request type!";
        }
    }
};

#endif  // PROJECT_CONTROLPARAMETERINTERFACE_H
