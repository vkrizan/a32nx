fn main() -> Result<(), Box<dyn std::error::Error>> {
    msfs::StandaloneModule::simulate(autothrottle::module)?;
    Ok(())
}
