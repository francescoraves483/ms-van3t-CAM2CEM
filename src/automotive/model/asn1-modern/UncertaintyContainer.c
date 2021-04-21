/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CEM-PDU-Descriptions"
 * 	found in "CEM v1.2.2.asn"
 */

#include "UncertaintyContainer.h"

asn_TYPE_member_t asn_MBR_UncertaintyContainer_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct UncertaintyContainer, pseudorangeUncertainty),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_GPSDataUncertainty,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"pseudorangeUncertainty"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct UncertaintyContainer, carrierPhaseUncertainty),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_GPSDataUncertainty,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"carrierPhaseUncertainty"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct UncertaintyContainer, dopplerUncertainty),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_GPSDataUncertainty,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"dopplerUncertainty"
		},
};
static const ber_tlv_tag_t asn_DEF_UncertaintyContainer_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_UncertaintyContainer_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* pseudorangeUncertainty */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* carrierPhaseUncertainty */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 } /* dopplerUncertainty */
};
asn_SEQUENCE_specifics_t asn_SPC_UncertaintyContainer_specs_1 = {
	sizeof(struct UncertaintyContainer),
	offsetof(struct UncertaintyContainer, _asn_ctx),
	asn_MAP_UncertaintyContainer_tag2el_1,
	3,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_UncertaintyContainer = {
	"UncertaintyContainer",
	"UncertaintyContainer",
	&asn_OP_SEQUENCE,
	asn_DEF_UncertaintyContainer_tags_1,
	sizeof(asn_DEF_UncertaintyContainer_tags_1)
		/sizeof(asn_DEF_UncertaintyContainer_tags_1[0]), /* 1 */
	asn_DEF_UncertaintyContainer_tags_1,	/* Same as above */
	sizeof(asn_DEF_UncertaintyContainer_tags_1)
		/sizeof(asn_DEF_UncertaintyContainer_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_UncertaintyContainer_1,
	3,	/* Elements count */
	&asn_SPC_UncertaintyContainer_specs_1	/* Additional specs */
};

