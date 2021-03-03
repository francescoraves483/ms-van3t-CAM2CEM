/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CEM-PDU-Descriptions"
 * 	found in "CEM v1.0.0.asn"
 */

#ifndef	_DifferentialMandatoryContainer_H_
#define	_DifferentialMandatoryContainer_H_


#include "asn_application.h"

/* Including external dependencies */
#include "ConstellationID.h"
#include "SatellitePRN.h"
#include "DifferentialPseudorange.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* DifferentialMandatoryContainer */
typedef struct DifferentialMandatoryContainer {
	ConstellationID_t	 constellationID;
	SatellitePRN_t	 satellitePRN;
	DifferentialPseudorange_t	 differentialPseudorange;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} DifferentialMandatoryContainer_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_DifferentialMandatoryContainer;
extern asn_SEQUENCE_specifics_t asn_SPC_DifferentialMandatoryContainer_specs_1;
extern asn_TYPE_member_t asn_MBR_DifferentialMandatoryContainer_1[3];

#ifdef __cplusplus
}
#endif

#endif	/* _DifferentialMandatoryContainer_H_ */
#include "asn_internal.h"
