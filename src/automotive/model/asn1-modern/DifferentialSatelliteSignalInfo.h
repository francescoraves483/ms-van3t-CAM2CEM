/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CEM-PDU-Descriptions"
 * 	found in "CEM v1.0.1.asn"
 */

#ifndef	_DifferentialSatelliteSignalInfo_H_
#define	_DifferentialSatelliteSignalInfo_H_


#include "asn_application.h"

/* Including external dependencies */
#include "DifferentialMandatoryContainer.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct DifferentialOptionalContainer;

/* DifferentialSatelliteSignalInfo */
typedef struct DifferentialSatelliteSignalInfo {
	DifferentialMandatoryContainer_t	 differentialMandatoryContainer;
	struct DifferentialOptionalContainer	*differentialOptionalContainer	/* OPTIONAL */;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} DifferentialSatelliteSignalInfo_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_DifferentialSatelliteSignalInfo;
extern asn_SEQUENCE_specifics_t asn_SPC_DifferentialSatelliteSignalInfo_specs_1;
extern asn_TYPE_member_t asn_MBR_DifferentialSatelliteSignalInfo_1[2];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "DifferentialOptionalContainer.h"

#endif	/* _DifferentialSatelliteSignalInfo_H_ */
#include "asn_internal.h"
