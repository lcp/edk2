#!/bin/sh

cd openssl-1.0.2h
cp ../opensslconf.h           crypto
mkdir -p                      include/openssl
cp e_os2.h                    include/openssl
cp crypto/crypto.h            include/openssl
cp crypto/opensslv.h          include/openssl
cp crypto/opensslconf.h       include/openssl
cp crypto/ebcdic.h            include/openssl
cp crypto/symhacks.h          include/openssl
cp crypto/ossl_typ.h          include/openssl
cp crypto/objects/objects.h   include/openssl
cp crypto/objects/obj_mac.h   include/openssl
cp crypto/md4/md4.h           include/openssl
cp crypto/md5/md5.h           include/openssl
cp crypto/sha/sha.h           include/openssl
cp crypto/mdc2/mdc2.h         include/openssl
cp crypto/hmac/hmac.h         include/openssl
cp crypto/ripemd/ripemd.h     include/openssl
cp crypto/whrlpool/whrlpool.h include/openssl
cp crypto/des/des.h           include/openssl
cp crypto/des/des_old.h       include/openssl
cp crypto/aes/aes.h           include/openssl
cp crypto/rc2/rc2.h           include/openssl
cp crypto/rc4/rc4.h           include/openssl
cp crypto/idea/idea.h         include/openssl
cp crypto/bf/blowfish.h       include/openssl
cp crypto/cast/cast.h         include/openssl
cp crypto/camellia/camellia.h include/openssl
cp crypto/seed/seed.h         include/openssl
cp crypto/modes/modes.h       include/openssl
cp crypto/bn/bn.h             include/openssl
cp crypto/ec/ec.h             include/openssl
cp crypto/rsa/rsa.h           include/openssl
cp crypto/dsa/dsa.h           include/openssl
cp crypto/ecdsa/ecdsa.h       include/openssl
cp crypto/dh/dh.h             include/openssl
cp crypto/ecdh/ecdh.h         include/openssl
cp crypto/dso/dso.h           include/openssl
cp crypto/engine/engine.h     include/openssl
cp crypto/buffer/buffer.h     include/openssl
cp crypto/bio/bio.h           include/openssl
cp crypto/stack/stack.h       include/openssl
cp crypto/stack/safestack.h   include/openssl
cp crypto/lhash/lhash.h       include/openssl
cp crypto/rand/rand.h         include/openssl
cp crypto/err/err.h           include/openssl
cp crypto/evp/evp.h           include/openssl
cp crypto/asn1/asn1.h         include/openssl
cp crypto/asn1/asn1_mac.h     include/openssl
cp crypto/asn1/asn1t.h        include/openssl
cp crypto/pem/pem.h           include/openssl
cp crypto/pem/pem2.h          include/openssl
cp crypto/x509/x509.h         include/openssl
cp crypto/x509/x509_vfy.h     include/openssl
cp crypto/x509v3/x509v3.h     include/openssl
cp crypto/conf/conf.h         include/openssl
cp crypto/conf/conf_api.h     include/openssl
cp crypto/txt_db/txt_db.h     include/openssl
cp crypto/pkcs7/pkcs7.h       include/openssl
cp crypto/pkcs12/pkcs12.h     include/openssl
cp crypto/comp/comp.h         include/openssl
cp crypto/ocsp/ocsp.h         include/openssl
cp crypto/ui/ui.h             include/openssl
cp crypto/ui/ui_compat.h      include/openssl
cp crypto/krb5/krb5_asn.h     include/openssl
cp crypto/cms/cms.h           include/openssl
cp crypto/pqueue/pqueue.h     include/openssl
cp crypto/ts/ts.h             include/openssl
cp crypto/srp/srp.h           include/openssl
cp crypto/cmac/cmac.h         include/openssl
cp ssl/ssl.h                  include/openssl
cp ssl/ssl2.h                 include/openssl
cp ssl/ssl3.h                 include/openssl
cp ssl/ssl23.h                include/openssl
cp ssl/tls1.h                 include/openssl
cp ssl/dtls1.h                include/openssl
cp ssl/kssl.h                 include/openssl
cp ssl/srtp.h                 include/openssl
cd ..
