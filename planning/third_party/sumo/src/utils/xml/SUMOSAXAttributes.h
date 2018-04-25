/****************************************************************************/
/// @file    SUMOSAXAttributes.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Fri, 30 Mar 2007
/// @version $Id: SUMOSAXAttributes.h 23150 2017-02-27 12:08:30Z behrisch $
///
// Encapsulated SAX-Attributes
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright (C) 2007-2017 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/
#ifndef SUMOSAXAttributes_h
#define SUMOSAXAttributes_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <vector>
#include <set>

#include <utils/common/SUMOTime.h>
#include <utils/common/ToString.h>
#include <utils/common/UtilExceptions.h>
#include "SUMOXMLDefinitions.h"


// ===========================================================================
// class declarations
// ===========================================================================
class PositionVector;
class Boundary;
class RGBColor;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class SUMOSAXAttributes
 * @brief Encapsulated SAX-Attributes
 *
 * This class is an interface for using encapsulated SAX-attributes.
 * Encapsulation is done to allow a common acces without the need to
 *  import all the Xerces-definitions.
 */
class SUMOSAXAttributes {
public:
    /* @brief Constructor
     * @param[in] tagName The name of the parsed object type; used for error message generation
     */
    SUMOSAXAttributes(const std::string& objectType);


    /// @brief Destructor
    virtual ~SUMOSAXAttributes() { }


    /** @brief Tries to read given attribute assuming it is an int
     *
     * If an error occurs (the attribute is not there, it is not numeric), "ok" is
     *  set to false. If report is true an error message is written to MsgHandler::getErrorInstance.
     *
     * If the value could be read, "ok" is not changed, and the value is returned.
     *
     * @param[in] attr The id of the attribute to read
     * @param[in] objectid The name of the parsed object; used for error message generation
     * @param[out] ok Whether the value could be read
     * @param[in] report Whether errors shall be written to msg handler's error instance
     * @return The read value if given and correct; -1 if an error occured
     */
    template <typename T>
    T get(int attr, const char* objectid, bool& ok, bool report = true) const;


    /** @brief Tries to read given attribute assuming it is an int
     *
     * If the attribute is not existing in the current element, the default value is returned.
     * If an error occurs on parsing (the attribute is empty, it is not numeric), "ok" is
     *  set to false. If report is true an error message is written to MsgHandler::getErrorInstance.
     *
     * If the value could be read, "ok" is not changed, and the value is returned.
     *
     * @param[in] attr The id of the attribute to read
     * @param[in] objectid The name of the parsed object; used for error message generation
     * @param[out] ok Whether the value could be read
     * @param[in] defaultValue The value to return if the attribute is not within the element
     * @param[in] report Whether errors shall be written to msg handler's error instance
     * @return The read value if given and correct; the default value if the attribute does not exist;  -1 if an error occured
     */
    template <typename T>
    T getOpt(int attr, const char* objectid, bool& ok, T defaultValue, bool report = true) const;


    /** @brief Tries to read given attribute assuming it is a SUMOTime
     *
     * If an error occurs (the attribute is not there, it is not numeric), "ok" is
     *  set to false and an error message is written to MsgHandler::getErrorInstance.
     *
     * Otherwise, "ok" is not changed.
     *
     * In dependence to the used time representation, either get<int> or get<double>
     *  is used.
     *
     * @param[in] attr The id of the attribute to read
     * @param[in] objectid The name of the parsed object; used for error message generation
     * @param[out] ok Whether the value could be read
     * @param[in] report Whether errors shall be written to msg handler's error instance
     * @return The read value if given and correct; -1 if an error occured
     */
    SUMOTime getSUMOTimeReporting(int attr, const char* objectid, bool& ok,
                                  bool report = true) const;



    /** @brief Tries to read given attribute assuming it is a SUMOTime
     *
     * If the attribute is not existing in the current element, the default value is returned.
     * If an error occurs on parsing (the attribute is empty, it is not numeric), "ok" is
     *  set to false. If report is true an error message is written to MsgHandler::getErrorInstance.
     *
     * Otherwise, "ok" is not changed.
     *
     * In dependence to the used time representation, either get<int> or get<double>
     *  is used.
     *
     * @param[in] attr The id of the attribute to read
     * @param[in] objectid The name of the parsed object; used for error message generation
     * @param[out] ok Whether the value could be read
     * @param[in] defaultValue The value to return if the attribute is not within the element
     * @param[in] report Whether errors shall be written to msg handler's error instance
     * @return The read value if given and correct; the default value if the attribute does not exist;  -1 if an error occured
     */
    SUMOTime getOptSUMOTimeReporting(int attr, const char* objectid, bool& ok,
                                     SUMOTime defaultValue, bool report = true) const;



    /// @name virtual methods for retrieving attribute values
    /// @{

    /** @brief Returns the information whether the named (by its enum-value) attribute is within the current list
     *
     * @param[in] id The id of the attribute to search for
     * @return Whether the attribute is within the attributes
     */
    virtual bool hasAttribute(int id) const = 0;


    /** @brief Returns the information whether the named attribute is within the current list
     *
     * @param[in] id The name of the attribute to search for
     * @return Whether the named attribute is within the attributes
     */
    virtual bool hasAttribute(const std::string& id) const = 0;


    /**
     * @brief Returns the bool-value of the named (by its enum-value) attribute
     *
     * Tries to retrieve the attribute from the the attribute list. The retrieved
     *  attribute  (which may be 0) is then parsed using TplConvert<XMLCh>::_2bool.
     *  If the attribute is empty or ==0, TplConvert<XMLCh>::_2bool throws an
     *  EmptyData-exception which is passed.
     * If the value can not be parsed to a bool, TplConvert<XMLCh>::_2bool throws a
     *  BoolFormatException-exception which is passed.
     *
     * @param[in] id The id of the attribute to return the value of
     * @return The attribute's value as a bool, if it could be read and parsed
     * @exception EmptyData If the attribute is not known or the attribute value is an empty string
     * @exception BoolFormatException If the attribute value can not be parsed to a bool
     */
    virtual bool getBool(int id) const = 0;

    /**
     * @brief Returns the int-value of the named (by its enum-value) attribute
     *
     * Tries to retrieve the attribute from the the attribute list. The retrieved
     *  attribute  (which may be 0) is then parsed using TplConvert<XMLCh>::_2int.
     *  If the attribute is empty or ==0, TplConvert<XMLCh>::_2int throws an
     *  EmptyData-exception which is passed.
     * If the value can not be parsed to an int, TplConvert<XMLCh>::_2int throws a
     *  NumberFormatException-exception which is passed.
     *
     * @param[in] id The id of the attribute to return the value of
     * @return The attribute's value as an int, if it could be read and parsed
     * @exception EmptyData If the attribute is not known or the attribute value is an empty string
     * @exception NumberFormatException If the attribute value can not be parsed to an int
     */
    virtual int getInt(int id) const = 0;


    /**
     * @brief Returns the long-value of the named (by its enum-value) attribute
     *
     * Tries to retrieve the attribute from the the attribute list. The retrieved
     *  attribute  (which may be 0) is then parsed using TplConvert<XMLCh>::_2long.
     *  If the attribute is empty or ==0, TplConvert<XMLCh>::_2long throws an
     *  EmptyData-exception which is passed.
     * If the value can not be parsed to a long, TplConvert<XMLCh>::_2long throws a
     *  NumberFormatException-exception which is passed.
     *
     * @param[in] id The id of the attribute to return the value of
     * @return The attribute's value as an int, if it could be read and parsed
     * @exception EmptyData If the attribute is not known or the attribute value is an empty string
     * @exception NumberFormatException If the attribute value can not be parsed to an int
     */
    virtual long long int getLong(int id) const = 0;


    /**
     * @brief Returns the string-value of the named (by its enum-value) attribute
     *
     * Tries to retrieve the attribute from the the attribute list. The retrieved
     *  attribute  (which may be 0) is then parsed using TplConvert<XMLCh>::_2str.
     *  If the attribute is ==0, TplConvert<XMLCh>::_2str throws an
     *  EmptyData-exception which is passed.
     *
     * @param[in] id The id of the attribute to return the value of
     * @return The attribute's value as a string, if it could be read and parsed
     * @exception EmptyData If the attribute is not known or the attribute value is an empty string
     */
    virtual std::string getString(int id) const = 0;


    /**
     * @brief Returns the string-value of the named (by its enum-value) attribute
     *
     * Tries to retrieve the attribute from the the attribute list. The retrieved
     *  attribute  (which may be 0) is then parsed using TplConvert<XMLCh>::_2strSec.
     *  If the attribute is ==0, TplConvert<XMLCh>::_2strSec returns the default value.
     *
     * @param[in] id The id of the attribute to return the value of
     * @param[in] def The default value to return if the attribute is not in attributes
     * @return The attribute's value as a string, if it could be read and parsed
     * @exception EmptyData If the attribute is not known or the attribute value is an empty string
     */
    virtual std::string getStringSecure(int id,
                                        const std::string& def) const = 0;


    /**
     * @brief Returns the double-value of the named (by its enum-value) attribute
     *
     * Tries to retrieve the attribute from the the attribute list. The retrieved
     *  attribute  (which may be 0) is then parsed using TplConvert<XMLCh>::_2double.
     *  If the attribute is empty or ==0, TplConvert<XMLCh>::_2double throws an
     *  EmptyData-exception which is passed.
     * If the value can not be parsed to a double, TplConvert<XMLCh>::_2double throws a
     *  NumberFormatException-exception which is passed.
     *
     * @param[in] id The id of the attribute to return the value of
     * @return The attribute's value as a float, if it could be read and parsed
     * @exception EmptyData If the attribute is not known or the attribute value is an empty string
     * @exception NumberFormatException If the attribute value can not be parsed to an double
     */
    virtual double getFloat(int id) const = 0;


    /**
     * @brief Returns the double-value of the named attribute
     *
     * Tries to retrieve the attribute from the the attribute list. The retrieved
     *  attribute  (which may be 0) is then parsed using TplConvert<XMLCh>::_2double.
     *  If the attribute is empty or ==0, TplConvert<XMLCh>::_2double throws an
     *  EmptyData-exception which is passed.
     * If the value can not be parsed to a double, TplConvert<XMLCh>::_2double throws a
     *  NumberFormatException-exception which is passed.
     *
     * @param[in] id The name of the attribute to return the value of
     * @return The attribute's value as a float, if it could be read and parsed
     * @exception EmptyData If the attribute is not known or the attribute value is an empty string
     * @exception NumberFormatException If the attribute value can not be parsed to an double
     */
    virtual double getFloat(const std::string& id) const = 0;


    /**
     * @brief Returns the string-value of the named (by its enum-value) attribute
     *
     * Tries to retrieve the attribute from the the attribute list.
     *  If the attribute is ==0, TplConvert<XMLCh>::_2strSec returns the default value.
     * @param[in] id The name of the attribute to return the value of
     * @param[in] def The default value to return if the attribute is not in attributes
     * @return The attribute's value as a string, if it could be read and parsed
     */
    virtual std::string getStringSecure(const std::string& id,
                                        const std::string& def) const = 0;


    /**
     * @brief Returns the value of the named attribute
     *
     * Tries to retrieve the attribute from the the attribute list.
     * @return The attribute's value as a string, if it could be read and parsed
     */
    virtual SumoXMLEdgeFunc getEdgeFunc(bool& ok) const = 0;


    /**
     * @brief Returns the value of the named attribute
     *
     * Tries to retrieve the attribute from the the attribute list.
     * @return The attribute's value as a string, if it could be read and parsed
     */
    virtual SumoXMLNodeType getNodeType(bool& ok) const = 0;


    /**
     * @brief Returns the value of the named attribute
     *
     * Tries to retrieve the attribute from the the attribute list.
     * @return The attribute's value as a RGBColor, if it could be read and parsed
     */
    virtual RGBColor getColor() const = 0;


    /** @brief Tries to read given attribute assuming it is a PositionVector
     *
     * @param[in] attr The id of the attribute to read
     * @return The read value if given and not empty; empty position vector if an error occured
     */
    virtual PositionVector getShape(int attr) const = 0;

    /** @brief Tries to read given attribute assuming it is a Boundary
     *
     * @param[in] attr The id of the attribute to read
     * @return The read value if given and not empty; empty Boundary if an error occured
     */
    virtual Boundary getBoundary(int attr) const = 0;

    /** @brief Tries to read given attribute assuming it is a string vector
     *
     * @param[in] attr The id of the attribute to read
     * @return The read value if given and not empty; empty vector if an error occured
     */
    virtual std::vector<std::string> getStringVector(int attr) const = 0;
    //}


    /** @brief Converts the given attribute id into a man readable string
     *
     * @param[in] attr The id of the attribute to return the name of
     * @return The name of the described attribute
     */
    virtual std::string getName(int attr) const = 0;


    /** @brief Prints all attribute names and values into the given stream
     *
     * @param[in] os The stream to use
     */
    virtual void serialize(std::ostream& os) const = 0;


    /// @brief return the objecttype to which these attributes belong
    const std::string& getObjectType() const {
        return myObjectType;
    }


    friend std::ostream& operator<<(std::ostream& os, const SUMOSAXAttributes& src);

    /// @brief return a new deep-copy attributes object
    virtual SUMOSAXAttributes* clone() const = 0;

    /** @brief The encoding of parsed strings */
    static const std::string ENCODING;


    /** @brief Splits the given string
     *
     * Spaces, ",", and ";" are assumed to be separator characters.
     * Though, in the case a "," or a ";" occurs, a warning is generated (once).
     *
     * @param[in] def The string to split
     * @param[out] into The vector to fill
     */
    static void parseStringVector(const std::string& def, std::vector<std::string>& into);


    /** @brief Splits the given string, stores it in a set
     *
     * Spaces, ",", and ";" are assumed to be separator characters.
     * Though, in the case a "," or a ";" occurs, a warning is generated (once).
     *
     * @param[in] def The string to split
     * @param[out] into The set to fill
     */
    static void parseStringSet(const std::string& def, std::set<std::string>& into);


protected:
    template <typename T> T getInternal(const int attr) const;
    void emitUngivenError(const std::string& attrname, const char* objectid) const;
    void emitEmptyError(const std::string& attrname, const char* objectid) const;
    void emitFormatError(const std::string& attrname, const std::string& type, const char* objectid) const;

private:
    /// @brief Information whether the usage of a deprecated divider was reported
    static bool myHaveInformedAboutDeprecatedDivider;

private:
    /// @brief Invalidated copy constructor.
    SUMOSAXAttributes(const SUMOSAXAttributes& src);

    /// @brief Invalidated assignment operator.
    SUMOSAXAttributes& operator=(const SUMOSAXAttributes& src);

    /// @brief the object type to use in error reporting
    std::string myObjectType;

};


inline std::ostream& operator<<(std::ostream& os, const SUMOSAXAttributes& src) {
    src.serialize(os);
    return os;
}


template<typename X> struct invalid_return {
    static const X value;
    static const std::string type;
};


template <typename T>
T SUMOSAXAttributes::get(int attr, const char* objectid,
                         bool& ok, bool report) const {
    if (!hasAttribute(attr)) {
        if (report) {
            emitUngivenError(getName(attr), objectid);
        }
        ok = false;
        return invalid_return<T>::value;
    }
    try {
        return getInternal<T>(attr);
    } catch (FormatException&) {
        if (report) {
            emitFormatError(getName(attr), "of type " + invalid_return<T>::type, objectid);
        }
    } catch (EmptyData&) {
        if (report) {
            emitEmptyError(getName(attr), objectid);
        }
    }
    ok = false;
    return invalid_return<T>::value;
}


template <typename T>
T SUMOSAXAttributes::getOpt(int attr, const char* objectid,
                            bool& ok, T defaultValue, bool report) const {
    if (!hasAttribute(attr)) {
        return defaultValue;
    }
    try {
        return getInternal<T>(attr);
    } catch (FormatException&) {
        if (report) {
            emitFormatError(getName(attr), "of type " + invalid_return<T>::type, objectid);
        }
    } catch (EmptyData&) {
        if (report) {
            emitEmptyError(getName(attr), objectid);
        }
    }
    ok = false;
    return invalid_return<T>::value;
}


#endif

/****************************************************************************/

