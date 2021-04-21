/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CEM-PDU-Descriptions"
 * 	found in "CEM v1.2.2.asn"
 */

#ifndef	_DifferentialMicroframe_H_
#define	_DifferentialMicroframe_H_


#include "asn_application.h"

/* Including external dependencies */
#include "FullPrecisionID.h"
#include "DifferentialID.h"
#include "CemTimestamp.h"
#include "asn_SEQUENCE_OF.h"
#include "constr_SEQUENCE_OF.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct DifferentialSatelliteSignalInfo;

/* DifferentialMicroframe */
typedef struct DifferentialMicroframe {
	FullPrecisionID_t	 fullPrecisionID;
	DifferentialID_t	 differentialID;
	CemTimestamp_t	 cemTstamp;
	struct differentialSatelliteSignalInfo {
		A_SEQUENCE_OF(struct DifferentialSatelliteSignalInfo) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} differentialSatelliteSignalInfo;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} DifferentialMicroframe_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_DifferentialMicroframe;
extern asn_SEQUENCE_specifics_t asn_SPC_DifferentialMicroframe_specs_1;
extern asn_TYPE_member_t asn_MBR_DifferentialMicroframe_1[4];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "DifferentialSatelliteSignalInfo.h"

#endif	/* _DifferentialMicroframe_H_ */
#include "asn_internal.h"
