#ifndef __GAALET_GEOMETRIC_PRODUCT_H
#define __GAALET_GEOMETRIC_PRODUCT_H

#include "utility.h"

namespace gaalet
{

namespace gp
{

//list of element multiplications with the same result type
struct msl_null
{
   static const conf_t size = 0;

   template<typename element_t, class L, class R>
   GAALET_CUDA_HOST_DEVICE
   static element_t product_sum(const L&, const R&)
   {
      return 0.0;
   }
};

template<conf_t LC, conf_t RC, typename T>
struct multiplication_sum_list
{
   static const conf_t left = LC;
   static const conf_t right = RC;

   typedef T tail;

   static const conf_t size = T::size + 1;

   template<typename element_t, class L, class R>
   GAALET_CUDA_HOST_DEVICE
   static element_t product_sum(const L& l, const R& r)
   {
      return
         l.template element<left>()*r.template element<right>()
         *CanonicalReorderingSign<left, right>::value
         *((BitCount<(L::metric::signature_bitmap|R::metric::signature_bitmap)&(left&right)>::value % 2) ? -1 : 1)
         + tail::template product_sum<element_t>(l, r);
   }
};
template<conf_t LC, conf_t RC>
struct multiplication_sum_list<LC, RC, msl_null>
{
   static const conf_t left = LC;
   static const conf_t right = RC;

   typedef msl_null tail;

   static const conf_t size = 1;

   template<typename element_t, class L, class R>
   GAALET_CUDA_HOST_DEVICE
   static element_t product_sum(const L& l, const R& r)
   {
      return
         l.template element<left>()*r.template element<right>()
         *CanonicalReorderingSign<left, right>::value
         *((BitCount<(L::metric::signature_bitmap|R::metric::signature_bitmap)&(left&right)>::value % 2) ? -1 : 1);
   }
};

//list of multiplication_sum_list, with respect to result type
template<conf_t C, typename H, typename T>
struct multiplication_element_list
{
   static const conf_t conf = C;
   typedef H head;
   typedef T tail;

   typedef configuration_list<conf, typename T::clist> clist;

   static const conf_t size = T::size + 1;

   template<typename element_t, class L, class R>
   GAALET_CUDA_HOST_DEVICE
   static element_t product_sum(const L& l, const R& r)
   {
      return head::template product_sum<element_t>(l, r);
   }
};

struct mel_null
{
   static const conf_t conf = 0;
   typedef cl_null clist;

   static const conf_t size = 0;

   template<typename element_t, class L, class R>
   GAALET_CUDA_HOST_DEVICE
   static element_t product_sum(const L&, const R&)
   {
      return 0.0;
   }
};

//insert_element to multiplication_element_list
template<conf_t LC, conf_t RC, typename list> struct insert_element_to_melist;

template<conf_t LC, conf_t RC, typename list, int op>
struct insert_element_to_melist_op
{
   typedef multiplication_element_list<list::conf, typename list::head, typename insert_element_to_melist<LC, RC, typename list::tail>::melist> melist;
};

template<conf_t LC, conf_t RC, int op>
struct insert_element_to_melist_op<LC, RC, mel_null, op>
{
   typedef multiplication_element_list<LC^RC, multiplication_sum_list<LC, RC, msl_null>, mel_null> melist;
};
template<conf_t LC, conf_t RC>
struct insert_element_to_melist_op<LC, RC, mel_null, 0>
{
   typedef multiplication_element_list<LC^RC, multiplication_sum_list<LC, RC, msl_null>, mel_null> melist;
};

template<conf_t LC, conf_t RC, typename list>
struct insert_element_to_melist_op<LC, RC, list, 0>
{
   typedef multiplication_element_list<list::conf, multiplication_sum_list<LC, RC, typename list::head>, typename list::tail> melist;
};

template<conf_t LC, conf_t RC, typename list>
struct insert_element_to_melist_op<LC, RC, list, 1>
{
   typedef multiplication_element_list<LC^RC, multiplication_sum_list<LC, RC, msl_null>, list> melist;
};

template<conf_t LC, conf_t RC, typename list>
struct insert_element_to_melist
{
   static const int op = ((LC^RC)==list::conf) ? 0 : (((LC^RC)<list::conf) ? 1 : -1);
   typedef typename insert_element_to_melist_op<LC, RC, list, op>::melist melist;
};


//build_multiplication_element_list
template<typename L, typename R, typename CL=L>
struct build_multiplication_element_list
{
   typedef typename insert_element_to_melist<L::head, R::head, typename build_multiplication_element_list<typename L::tail, R, CL>::melist>::melist melist;
};
template<typename R, typename CL>
struct build_multiplication_element_list<cl_null, R, CL>
{
   typedef typename build_multiplication_element_list<CL, typename R::tail, CL>::melist melist;
};
template<typename L, typename CL>
struct build_multiplication_element_list<L, cl_null, CL>
{
   typedef mel_null melist;
};

//search melist
template<conf_t conf, typename list> struct search_conf_in_melist;

template<conf_t conf, typename list, bool fit>
struct search_conf_in_melist_op
{
   typedef typename search_conf_in_melist<conf, typename list::tail>::melist melist;
};

template<conf_t conf, typename list>
struct search_conf_in_melist_op<conf, list, true>
{
   typedef list melist;
};

template<conf_t conf, bool fit>
struct search_conf_in_melist_op<conf, mel_null, fit>
{
   typedef mel_null melist;
};
template<conf_t conf>
struct search_conf_in_melist_op<conf, mel_null, true>
{
   typedef mel_null melist;
};

template<conf_t conf, typename list>
struct search_conf_in_melist
{
   static const bool fit = (conf==list::conf);
   typedef typename search_conf_in_melist_op<conf, list, fit>::melist melist;
};

}  //end namespace gp

template<class L, class R>
struct geometric_product : public expression<geometric_product<L, R> >
{
   typedef typename gp::build_multiplication_element_list<typename L::clist, typename R::clist>::melist melist;
   typedef typename melist::clist clist;

   typedef typename metric_combination_traits<typename L::metric, typename R::metric>::metric metric;

   typedef typename element_type_combination_traits<typename L::element_t, typename R::element_t>::element_t element_t;

   geometric_product(const L& l_ , const R& r_)
      :  l(l_), r(r_)
   { }

   template<conf_t conf>
   GAALET_CUDA_HOST_DEVICE
   element_t element() const {
      return gp::search_conf_in_melist<conf, melist>::melist::template product_sum<element_t>(l, r);
   }

protected:
   const L& l;
   const R& r;
};

template<class A>
struct scalar_multivector_product : public expression<scalar_multivector_product<A> >
{
   typedef typename A::clist clist;

   typedef typename A::metric metric;

   typedef typename A::element_t element_t;

   scalar_multivector_product(const element_t& s_, const A& a_)
      :  s(s_), a(a_)
   { }

   template<conf_t conf>
   GAALET_CUDA_HOST_DEVICE
   element_t element() const {
      return s*a.template element<conf>();
   }

protected:
   const element_t& s;
   const A& a;
};

} //end namespace gaalet


template <class L, class R> inline
GAALET_CUDA_HOST_DEVICE
gaalet::geometric_product<L, R>
operator*(const gaalet::expression<L>& l, const gaalet::expression<R>& r) {
   return gaalet::geometric_product<L, R>(l, r);
}

template <class A> inline
GAALET_CUDA_HOST_DEVICE
gaalet::scalar_multivector_product<A>
operator*(const typename A::element_t& s, const gaalet::expression<A>& a) {
   return gaalet::scalar_multivector_product<A>(s, a);
}

template <class A> inline
GAALET_CUDA_HOST_DEVICE
gaalet::scalar_multivector_product<A>
operator*(const gaalet::expression<A>& a, const typename A::element_t& s) {
   return gaalet::scalar_multivector_product<A>(s, a);
}

#endif
