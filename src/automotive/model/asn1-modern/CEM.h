/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CEM-PDU-Descriptions"
 * 	found in "CEM v1.0.0.asn"
 */

#ifndef	_CEM_H_
#define	_CEM_H_


#include "asn_application.h"

/* Including external dependencies */
#include "ItsPduHeader.h"
#include "CoopEnhancement.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* CEM */
typedef struct CEM {
	ItsPduHeader_t	 header;
	CoopEnhancement_t	 cem;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} CEM_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_CEM;

#ifdef __cplusplus
}
#endif

#endif	/* _CEM_H_ */
#include "asn_internal.h"
