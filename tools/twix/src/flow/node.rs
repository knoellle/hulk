trait Node {
    type Output;

    fn update(&mut self);

    fn get_data(&self) -> &Self::Output;
}

struct ConstData<T> {
    value: T,
}

impl<T> Node for ConstData<T> {
    type Output = T;

    fn update(&mut self) {
        // intentionally left blank
    }

    fn get_data(&self) -> &Self::Output {
        &self.value
    }
}

struct GenericProcessor<'source, F, I, O>
where
    F: FnMut(&I::Output) -> O,
    I: Node,
{
    source: &'source I,
    closure: F,
}

impl<'source, F, I, O> Node for GenericProcessor<'source, F, I, O>
where
    F: FnMut(&I::Output) -> O,
    I: Node,
{
    type Output = O;

    fn update(&mut self) {
        todo!()
    }

    fn get_data(&self) -> &Self::Output {
        todo!()
    }
}
