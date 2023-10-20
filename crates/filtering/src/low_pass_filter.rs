use std::{
    f32::consts::PI,
    ops::{Add, Mul, Sub},
};

use serde::{Deserialize, Serialize};

#[derive(Serialize, Clone, Default, Debug)]
pub struct LowPassFilter<State: Serialize> {
    smoothing_factor: f32,
    state: State,
}

impl<State> LowPassFilter<State>
where
    State: Copy + Add<Output = State> + Sub<Output = State> + Mul<f32, Output = State> + Serialize,
{
    pub fn with_smoothing_factor(initial_state: State, smoothing_factor: f32) -> Self {
        Self {
            smoothing_factor,
            state: initial_state,
        }
    }

    pub fn with_cutoff(initial_state: State, cutoff_frequency: f32, sampling_rate: f32) -> Self {
        let rc = 1.0 / (cutoff_frequency * 2.0 * PI);
        let dt = 1.0 / sampling_rate;
        let smoothing_factor = dt / (rc + dt);
        Self {
            smoothing_factor,
            state: initial_state,
        }
    }

    pub fn update(&mut self, value: State) {
        self.state = self.state + (value - self.state) * self.smoothing_factor;
    }

    pub fn state(&self) -> State {
        self.state
    }

    pub fn reset(&mut self, state: State) {
        self.state = state;
    }
}

#[cfg(test)]
mod test {
    use nalgebra::{vector, Vector3};

    use super::*;

    #[test]
    fn deserialize_in_place_correctly() {
        let mut filter = LowPassFilter::with_smoothing_factor(vector![0.0, 0.2, 3.0], 0.5);
        let initial_state = serde_json::to_value(&filter).unwrap();
        filter.update(vector![5.0, -0.5, 0.0]);
        filter.smoothing_factor = 3.0;
        let changed_state = serde_json::to_value(&filter).unwrap();

        LowPassFilter::deserialize_in_place(&initial_state, &mut filter).unwrap();
        let restored_state = serde_json::to_value(&filter).unwrap();

        filter.update(vector![5.0, -0.5, 0.0]);
        filter.smoothing_factor = 3.0;
        let changed_state2 = serde_json::to_value(&filter).unwrap();

        assert_eq!(initial_state, restored_state);

        assert_eq!(changed_state, changed_state2);
    }

    #[test]
    fn deserialize_correctly() {
        let filter = LowPassFilter::with_smoothing_factor(vector![0.0, 0.2, 3.0], 0.5);
        let initial_state = serde_json::to_value(filter).unwrap();

        let new_filter: LowPassFilter<Vector3<f32>> =
            serde_json::from_value(initial_state.clone()).unwrap();
        let new_state = serde_json::to_value(new_filter).unwrap();

        assert_eq!(initial_state, new_state);
    }
}

#[doc(hidden)]
#[allow(non_upper_case_globals, unused_attributes, unused_qualifications)]
const _: () = {
    #[allow(unused_extern_crates, clippy::useless_attribute)]
    extern crate serde as _serde;
    #[automatically_derived]
    impl<'de, State: Serialize> _serde::Deserialize<'de> for LowPassFilter<State>
    where
        State: _serde::Deserialize<'de>,
    {
        fn deserialize<__D>(__deserializer: __D) -> _serde::__private::Result<Self, __D::Error>
        where
            __D: _serde::Deserializer<'de>,
        {
            #[allow(non_camel_case_types)]
            #[doc(hidden)]
            enum __Field {
                __field0,
                __field1,
                __ignore,
            }
            #[doc(hidden)]
            struct __FieldVisitor;
            impl<'de> _serde::de::Visitor<'de> for __FieldVisitor {
                type Value = __Field;
                fn expecting(
                    &self,
                    __formatter: &mut _serde::__private::Formatter,
                ) -> _serde::__private::fmt::Result {
                    _serde::__private::Formatter::write_str(__formatter, "field identifier")
                }
                fn visit_u64<__E>(self, __value: u64) -> _serde::__private::Result<Self::Value, __E>
                where
                    __E: _serde::de::Error,
                {
                    match __value {
                        0u64 => _serde::__private::Ok(__Field::__field0),
                        1u64 => _serde::__private::Ok(__Field::__field1),
                        _ => _serde::__private::Ok(__Field::__ignore),
                    }
                }
                fn visit_str<__E>(
                    self,
                    __value: &str,
                ) -> _serde::__private::Result<Self::Value, __E>
                where
                    __E: _serde::de::Error,
                {
                    match __value {
                        "smoothing_factor" => _serde::__private::Ok(__Field::__field0),
                        "state" => _serde::__private::Ok(__Field::__field1),
                        _ => _serde::__private::Ok(__Field::__ignore),
                    }
                }
                fn visit_bytes<__E>(
                    self,
                    __value: &[u8],
                ) -> _serde::__private::Result<Self::Value, __E>
                where
                    __E: _serde::de::Error,
                {
                    match __value {
                        b"smoothing_factor" => _serde::__private::Ok(__Field::__field0),
                        b"state" => _serde::__private::Ok(__Field::__field1),
                        _ => _serde::__private::Ok(__Field::__ignore),
                    }
                }
            }
            impl<'de> _serde::Deserialize<'de> for __Field {
                #[inline]
                fn deserialize<__D>(
                    __deserializer: __D,
                ) -> _serde::__private::Result<Self, __D::Error>
                where
                    __D: _serde::Deserializer<'de>,
                {
                    _serde::Deserializer::deserialize_identifier(__deserializer, __FieldVisitor)
                }
            }
            #[doc(hidden)]
            struct __Visitor<'de, State: Serialize>
            where
                State: _serde::Deserialize<'de>,
            {
                marker: _serde::__private::PhantomData<LowPassFilter<State>>,
                lifetime: _serde::__private::PhantomData<&'de ()>,
            }
            impl<'de, State: Serialize> _serde::de::Visitor<'de> for __Visitor<'de, State>
            where
                State: _serde::Deserialize<'de>,
            {
                type Value = LowPassFilter<State>;
                fn expecting(
                    &self,
                    __formatter: &mut _serde::__private::Formatter,
                ) -> _serde::__private::fmt::Result {
                    _serde::__private::Formatter::write_str(__formatter, "struct LowPassFilter")
                }
                #[inline]
                fn visit_seq<__A>(
                    self,
                    mut __seq: __A,
                ) -> _serde::__private::Result<Self::Value, __A::Error>
                where
                    __A: _serde::de::SeqAccess<'de>,
                {
                    let __field0 = match _serde::de::SeqAccess::next_element::<f32>(&mut __seq)? {
                        _serde::__private::Some(__value) => __value,
                        _serde::__private::None => {
                            return _serde::__private::Err(_serde::de::Error::invalid_length(
                                0usize,
                                &"struct LowPassFilter with 2 elements",
                            ))
                        }
                    };
                    let __field1 = match _serde::de::SeqAccess::next_element::<State>(&mut __seq)? {
                        _serde::__private::Some(__value) => __value,
                        _serde::__private::None => {
                            return _serde::__private::Err(_serde::de::Error::invalid_length(
                                1usize,
                                &"struct LowPassFilter with 2 elements",
                            ))
                        }
                    };
                    _serde::__private::Ok(LowPassFilter {
                        smoothing_factor: __field0,
                        state: __field1,
                    })
                }
                #[inline]
                fn visit_map<__A>(
                    self,
                    mut __map: __A,
                ) -> _serde::__private::Result<Self::Value, __A::Error>
                where
                    __A: _serde::de::MapAccess<'de>,
                {
                    let mut __field0: _serde::__private::Option<f32> = _serde::__private::None;
                    let mut __field1: _serde::__private::Option<State> = _serde::__private::None;
                    while let _serde::__private::Some(__key) =
                        _serde::de::MapAccess::next_key::<__Field>(&mut __map)?
                    {
                        match __key {
                            __Field::__field0 => {
                                if _serde::__private::Option::is_some(&__field0) {
                                    return _serde::__private::Err(
                                        <__A::Error as _serde::de::Error>::duplicate_field(
                                            "smoothing_factor",
                                        ),
                                    );
                                }
                                __field0 =
                                    _serde::__private::Some(_serde::de::MapAccess::next_value::<
                                        f32,
                                    >(
                                        &mut __map
                                    )?);
                            }
                            __Field::__field1 => {
                                if _serde::__private::Option::is_some(&__field1) {
                                    return _serde::__private::Err(
                                        <__A::Error as _serde::de::Error>::duplicate_field("state"),
                                    );
                                }
                                __field1 =
                                    _serde::__private::Some(_serde::de::MapAccess::next_value::<
                                        State,
                                    >(
                                        &mut __map
                                    )?);
                            }
                            _ => {
                                let _ = _serde::de::MapAccess::next_value::<_serde::de::IgnoredAny>(
                                    &mut __map,
                                )?;
                            }
                        }
                    }
                    let __field0 = match __field0 {
                        _serde::__private::Some(__field0) => __field0,
                        _serde::__private::None => {
                            _serde::__private::de::missing_field("smoothing_factor")?
                        }
                    };
                    let __field1 = match __field1 {
                        _serde::__private::Some(__field1) => __field1,
                        _serde::__private::None => _serde::__private::de::missing_field("state")?,
                    };
                    _serde::__private::Ok(LowPassFilter {
                        smoothing_factor: __field0,
                        state: __field1,
                    })
                }
            }
            #[doc(hidden)]
            const FIELDS: &'static [&'static str] = &["smoothing_factor", "state"];
            _serde::Deserializer::deserialize_struct(
                __deserializer,
                "LowPassFilter",
                FIELDS,
                __Visitor {
                    marker: _serde::__private::PhantomData::<LowPassFilter<State>>,
                    lifetime: _serde::__private::PhantomData,
                },
            )
        }
        fn deserialize_in_place<__D>(
            __deserializer: __D,
            __place: &mut Self,
        ) -> _serde::__private::Result<(), __D::Error>
        where
            __D: _serde::Deserializer<'de>,
        {
            #[allow(non_camel_case_types)]
            #[doc(hidden)]
            enum __Field {
                __field0,
                __field1,
                __ignore,
            }
            #[doc(hidden)]
            struct __FieldVisitor;
            impl<'de> _serde::de::Visitor<'de> for __FieldVisitor {
                type Value = __Field;
                fn expecting(
                    &self,
                    __formatter: &mut _serde::__private::Formatter,
                ) -> _serde::__private::fmt::Result {
                    _serde::__private::Formatter::write_str(__formatter, "field identifier")
                }
                fn visit_u64<__E>(self, __value: u64) -> _serde::__private::Result<Self::Value, __E>
                where
                    __E: _serde::de::Error,
                {
                    match __value {
                        0u64 => _serde::__private::Ok(__Field::__field0),
                        1u64 => _serde::__private::Ok(__Field::__field1),
                        _ => _serde::__private::Ok(__Field::__ignore),
                    }
                }
                fn visit_str<__E>(
                    self,
                    __value: &str,
                ) -> _serde::__private::Result<Self::Value, __E>
                where
                    __E: _serde::de::Error,
                {
                    match __value {
                        "smoothing_factor" => _serde::__private::Ok(__Field::__field0),
                        "state" => _serde::__private::Ok(__Field::__field1),
                        _ => _serde::__private::Ok(__Field::__ignore),
                    }
                }
                fn visit_bytes<__E>(
                    self,
                    __value: &[u8],
                ) -> _serde::__private::Result<Self::Value, __E>
                where
                    __E: _serde::de::Error,
                {
                    match __value {
                        b"smoothing_factor" => _serde::__private::Ok(__Field::__field0),
                        b"state" => _serde::__private::Ok(__Field::__field1),
                        _ => _serde::__private::Ok(__Field::__ignore),
                    }
                }
            }
            impl<'de> _serde::Deserialize<'de> for __Field {
                #[inline]
                fn deserialize<__D>(
                    __deserializer: __D,
                ) -> _serde::__private::Result<Self, __D::Error>
                where
                    __D: _serde::Deserializer<'de>,
                {
                    _serde::Deserializer::deserialize_identifier(__deserializer, __FieldVisitor)
                }
            }
            #[doc(hidden)]
            struct __Visitor<'de, 'place, State: Serialize + 'place>
            where
                State: _serde::Deserialize<'de>,
            {
                place: &'place mut LowPassFilter<State>,
                lifetime: _serde::__private::PhantomData<&'de ()>,
            }
            impl<'de, 'place, State: Serialize + 'place> _serde::de::Visitor<'de>
                for __Visitor<'de, 'place, State>
            where
                State: _serde::Deserialize<'de>,
            {
                type Value = ();
                fn expecting(
                    &self,
                    __formatter: &mut _serde::__private::Formatter,
                ) -> _serde::__private::fmt::Result {
                    _serde::__private::Formatter::write_str(__formatter, "struct LowPassFilter")
                }
                #[inline]
                fn visit_seq<__A>(
                    self,
                    mut __seq: __A,
                ) -> _serde::__private::Result<Self::Value, __A::Error>
                where
                    __A: _serde::de::SeqAccess<'de>,
                {
                    if let _serde::__private::None = _serde::de::SeqAccess::next_element_seed(
                        &mut __seq,
                        _serde::__private::de::InPlaceSeed(&mut self.place.smoothing_factor),
                    )? {
                        return _serde::__private::Err(_serde::de::Error::invalid_length(
                            0usize,
                            &"struct LowPassFilter with 2 elements",
                        ));
                    }
                    if let _serde::__private::None = _serde::de::SeqAccess::next_element_seed(
                        &mut __seq,
                        _serde::__private::de::InPlaceSeed(&mut self.place.state),
                    )? {
                        return _serde::__private::Err(_serde::de::Error::invalid_length(
                            1usize,
                            &"struct LowPassFilter with 2 elements",
                        ));
                    }
                    _serde::__private::Ok(())
                }
                #[inline]
                fn visit_map<__A>(
                    self,
                    mut __map: __A,
                ) -> _serde::__private::Result<Self::Value, __A::Error>
                where
                    __A: _serde::de::MapAccess<'de>,
                {
                    let mut __field0: bool = false;
                    let mut __field1: bool = false;
                    while let _serde::__private::Some(__key) =
                        _serde::de::MapAccess::next_key::<__Field>(&mut __map)?
                    {
                        match __key {
                            __Field::__field0 => {
                                if __field0 {
                                    return _serde::__private::Err(
                                        <__A::Error as _serde::de::Error>::duplicate_field(
                                            "smoothing_factor",
                                        ),
                                    );
                                }
                                _serde::de::MapAccess::next_value_seed(
                                    &mut __map,
                                    _serde::__private::de::InPlaceSeed(
                                        &mut self.place.smoothing_factor,
                                    ),
                                )?;
                                __field0 = true;
                            }
                            __Field::__field1 => {
                                if __field1 {
                                    return _serde::__private::Err(
                                        <__A::Error as _serde::de::Error>::duplicate_field("state"),
                                    );
                                }
                                _serde::de::MapAccess::next_value_seed(
                                    &mut __map,
                                    _serde::__private::de::InPlaceSeed(&mut self.place.state),
                                )?;
                                __field1 = true;
                            }
                            _ => {
                                let _ = _serde::de::MapAccess::next_value::<_serde::de::IgnoredAny>(
                                    &mut __map,
                                )?;
                            }
                        }
                    }
                    if !__field0 {
                        self.place.smoothing_factor =
                            _serde::__private::de::missing_field("smoothing_factor")?;
                    };
                    if !__field1 {
                        self.place.state = _serde::__private::de::missing_field("state")?;
                    };
                    _serde::__private::Ok(())
                }
            }
            #[doc(hidden)]
            const FIELDS: &'static [&'static str] = &["smoothing_factor", "state"];
            _serde::Deserializer::deserialize_struct(
                __deserializer,
                "LowPassFilter",
                FIELDS,
                __Visitor {
                    place: __place,
                    lifetime: _serde::__private::PhantomData,
                },
            )
        }
    }
};

