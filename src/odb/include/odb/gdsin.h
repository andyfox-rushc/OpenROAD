// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#pragma once

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <set>
#include <string>
#include <vector>

#include "odb/db.h"
#include "odb/gdsUtil.h"

namespace odb::gds {

class GDSReader
{
 public:
  GDSReader(utl::Logger* logger);

  /**
   * Reads a GDS file and returns a dbGDSLib object
   *
   * @param filename The path to the GDS file
   * @param db The database to store the GDS data
   * @return A dbGDSLib object containing the GDS data
   * @throws std::runtime_error if the file cannot be opened, or if the GDS is
   * corrupted
   */
  dbGDSLib* read_gds(const std::string& filename, dbDatabase* db);

 private:
  /**
   * Checks if the read record is the expected type
   *
   * Currently functions as an assert statement for the record type.
   * Used after readRecord() to check if the record type is the expected type.
   *
   * @param expect The expected record type
   * @throws std::runtime_error if the record type is not the expected type
   */
  void checkRType(RecordType expect);

  /**
   * Checks if the read record is the expected data type
   *
   * Currently functions as an assert statement for the data type.
   * Used after readRecord() to check if the data type is the expected type.
   * This function also checks if the data size is the expected size.
   *
   * @param eType The expected data type
   * @param eSize The expected data size
   * @throws std::runtime_error if the data type is not the expected type
   */
  void checkRData(DataType eType, size_t eSize);

  /**
   * Reads a real8 from _file
   *
   * NOTE: real8 is not the same as double. This conversion is not lossless.
   * @return The real8 read from _file, converted to a double
   */
  double readReal8();

  /** Reads an int32 from _file */
  int32_t readInt32();

  /** Reads an int16 from _file */
  int16_t readInt16();

  /** Reads an int8 from _file */
  int8_t readInt8();

  /**
   * Reads a record from _file and stores it in _r
   *
   * Reads the record type, data type, and length from _file.
   * The data is then read into the appropriate data vector.
   *
   * @return true if a record was read, false if the end of the file was reached
   * @throws std::runtime_error if the record is corrupted
   */
  bool readRecord();

  /** Parses a GDS Lib from the GDS file */
  bool processLib();

  /** Parses a GDS Structure from the GDS file */
  bool processStruct();

  /**
   * Parses a GDS Element from the GDS file
   *
   * @param str The GDS Structure to add the GDS Element to
   */
  bool processElement(dbGDSStructure* structure);

  // Specific element types, same as processElement
  dbGDSBoundary* processBoundary(dbGDSStructure* structure);
  dbGDSPath* processPath(dbGDSStructure* structure);
  dbGDSSRef* processSRef(dbGDSStructure* structure);
  dbGDSARef* processARef(dbGDSStructure* structure);
  dbGDSText* processText(dbGDSStructure* structure);
  dbGDSBox* processBox(dbGDSStructure* structure);
  void processNode();

  /**
   * Parses special attributes of a GDS Element
   *
   * @param elem The GDS Element to add the attributes to
   */
  template <typename T>
  void processPropAttr(T* elem);

  /**
   * Parses the XY data of a GDS Element
   *
   * @return The XY data
   */
  std::vector<Point> processXY();

  /**
   * Parses a GDS STrans from the GDS file
   * @return The parsed STrans
   */
  dbGDSSTrans processSTrans();

  /**
   * Parses a GDS Text Presentation from the GDS file
   * @return The parsed STrans
   */
  dbGDSTextPres processTextPres();

  /** Current filestream */
  std::ifstream _file;
  /** Most recently read record */
  record_t _r;
  /** Current ODB Database */
  dbDatabase* _db = nullptr;
  /** Current GDS Lib object */
  dbGDSLib* _lib = nullptr;
  /** An sref may refer to a structure that isn't yet built while
      reading the gds.  We will make an empty structure but it isn't
      yet defined.  We keep track of defined structures to catch any
      duplicates.*/
  std::set<dbGDSStructure*> _defined;

  utl::Logger* _logger{nullptr};
};

}  // namespace odb::gds
