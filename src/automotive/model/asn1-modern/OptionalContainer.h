/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CEM-PDU-Descriptions"
 * 	found in "CEM v1.0.0.asn"
 */

#ifndef	_OptionalContainer_H_
#define	_OptionalContainer_H_


#include "asn_application.h"

/* Including external dependencies */
#include "CarrierPhase.h"
#include "GPSDataUncertainty.h"
#include "Doppler.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* OptionalContainer */
typedef struct OptionalContainer {
	CarrierPhase_t	 carrierPhase;
	GPSDataUncertainty_t	 carrierPhaseUncertainty;
	Doppler_t	 doppler;
	GPSDataUncertainty_t	 dopplerUncertainty;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} OptionalContainer_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_OptionalContainer;
extern asn_SEQUENCE_specifics_t asn_SPC_OptionalContainer_specs_1;
extern asn_TYPE_member_t asn_MBR_OptionalContainer_1[4];

#ifdef __cplusplus
}
#endif

#endif	/* _OptionalContainer_H_ */
#include "asn_internal.h"
