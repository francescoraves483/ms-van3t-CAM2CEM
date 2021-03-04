/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CEM-PDU-Descriptions"
 * 	found in "CEM v1.0.1.asn"
 */

#include "DifferentialOptionalContainer.h"

asn_TYPE_member_t asn_MBR_DifferentialOptionalContainer_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct DifferentialOptionalContainer, differentialCarrierPhase),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_DifferentialCarrierPhase,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"differentialCarrierPhase"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct DifferentialOptionalContainer, differentialCarrierPhaseUncertainty),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_GPSDataUncertainty,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"differentialCarrierPhaseUncertainty"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct DifferentialOptionalContainer, differentialDoppler),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_DifferentialDoppler,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"differentialDoppler"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct DifferentialOptionalContainer, differentialDopplerUncertainty),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_GPSDataUncertainty,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"differentialDopplerUncertainty"
		},
};
static const ber_tlv_tag_t asn_DEF_DifferentialOptionalContainer_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_DifferentialOptionalContainer_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* differentialCarrierPhase */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* differentialCarrierPhaseUncertainty */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* differentialDoppler */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 } /* differentialDopplerUncertainty */
};
asn_SEQUENCE_specifics_t asn_SPC_DifferentialOptionalContainer_specs_1 = {
	sizeof(struct DifferentialOptionalContainer),
	offsetof(struct DifferentialOptionalContainer, _asn_ctx),
	asn_MAP_DifferentialOptionalContainer_tag2el_1,
	4,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_DifferentialOptionalContainer = {
	"DifferentialOptionalContainer",
	"DifferentialOptionalContainer",
	&asn_OP_SEQUENCE,
	asn_DEF_DifferentialOptionalContainer_tags_1,
	sizeof(asn_DEF_DifferentialOptionalContainer_tags_1)
		/sizeof(asn_DEF_DifferentialOptionalContainer_tags_1[0]), /* 1 */
	asn_DEF_DifferentialOptionalContainer_tags_1,	/* Same as above */
	sizeof(asn_DEF_DifferentialOptionalContainer_tags_1)
		/sizeof(asn_DEF_DifferentialOptionalContainer_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_DifferentialOptionalContainer_1,
	4,	/* Elements count */
	&asn_SPC_DifferentialOptionalContainer_specs_1	/* Additional specs */
};

