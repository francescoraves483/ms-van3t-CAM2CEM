/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CEM-PDU-Descriptions"
 * 	found in "ASN1Files/CEM v1.2.1.asn"
 */

#ifndef	_FullPrecisionInterframe_H_
#define	_FullPrecisionInterframe_H_


#include "asn_application.h"

/* Including external dependencies */
#include "FullPrecisionID.h"
#include "CemTimestamp.h"
#include "asn_SEQUENCE_OF.h"
#include "constr_SEQUENCE_OF.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct SatelliteSignalInfo;

/* FullPrecisionInterframe */
typedef struct FullPrecisionInterframe {
	FullPrecisionID_t	 fullPrecisionID;
	CemTimestamp_t	 cemTstamp;
	struct satelliteSignalInfo {
		A_SEQUENCE_OF(struct SatelliteSignalInfo) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} satelliteSignalInfo;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} FullPrecisionInterframe_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_FullPrecisionInterframe;
extern asn_SEQUENCE_specifics_t asn_SPC_FullPrecisionInterframe_specs_1;
extern asn_TYPE_member_t asn_MBR_FullPrecisionInterframe_1[3];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "SatelliteSignalInfo.h"

#endif	/* _FullPrecisionInterframe_H_ */
#include "asn_internal.h"
