<!DOCTYPE html>
<html lang="en-us">
  <head>
    <meta charset="UTF-8">
    <title>ER EExER Firmware by dhahaj</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <link rel="stylesheet" type="text/css" href="stylesheets/normalize.css" media="screen">
    <link href='https://fonts.googleapis.com/css?family=Open+Sans:400,700' rel='stylesheet' type='text/css'>
    <link rel="stylesheet" type="text/css" href="stylesheets/stylesheet.css" media="screen">
    <link rel="stylesheet" type="text/css" href="stylesheets/github-light.css" media="screen">
  </head>
  <body>
    <section class="page-header">
      <h1 class="project-name">ER EExER Firmware</h1>
      <h2 class="project-tagline">Latest Revision: v5.2</h2>
      <a href="https://github.com/dhahaj/ER_FIRMWARE" class="btn">View on GitHub</a>
      <a href="https://github.com/dhahaj/ER_FIRMWARE/zipball/master" class="btn">Download .zip</a>
      <a href="https://github.com/dhahaj/ER_FIRMWARE/tarball/master" class="btn">Download .tar.gz</a>
    </section>

    <section class="main-content">
<pre>
# PS8x FIRMWARE 

=======
1/27/2017: TODO - modify the preprocessor code to keeep the interrupt code and instean use the preprocessor to turn off the pin change interrupts on certain pins. 

>>>>>>> origin/master
REVISION HISTORY:
  8/21/2014: Release candidate version 5.1
  9/13/2015: Rev 5.2 to fix a bug in the timing code found from DVPR1 testing

<strong>Compilation Info:</strong>
  Define a symbol named EExER with a value of 1 to compile the code for the EExER build version. Set the value to 0 to build the standard ER build version.

<h3>Build Versions:</h3> *EExER* - This build essentially removes most of the features found in the ER code as it is not required. More specifically, no DIP setting features and trigger Relay1 before triggering the rest of the outputs.
 *ER* - This version is compiled to be the standard ER firmware with the ability to use the entire feature set (e.g. the DIP switch settings). It allows for Dependent/Independent operation on dual doors, Active High/Low outputs, and Toggling features. The DIP switches are interrupt driven so changes are implemented immediately and the system auto adjusts depending on its current state. The order of events upon a trigger event is as follows: 
<h3>Mode Definitions:</h3> *Dependent Mode:*  (INPUT1 | INPUT2) => (DOOR1 ACTIVE)->(DELAY 0.5s)->(DOOR2 ACTIVE)->(DELAY 0.5s)->(RELAY1 & RELAY2)->(DELAY DS1)->(RETURN)
 *Independent Mode:*  (INPUTn) => (DOORn ACTIVE)->(DELAY 0.5s)->(RELAYn)->(DELAY DSn)->(RETURN)
   <i><u>Operation in independent mode also requires the monitoring of the opposing door input during the delay time so that a triggering event is not missed. This is accomplished by controlling the timing and outputs outputs within the Timer1 Compare Interrupt Service Routine Vector (see interrupts.c).__</i></u>

<b>*EExER*</b> - This version is compiled for compatibility with the current delayed egress used in the 85-800. The pre-compiler bypasses all the DIP switch features as the delayed egress board controls the outputs. This version does not populate the DIP switch (DIP1) or Door 2 rotary encoder (DS2) as neither of these features will be used in this configuration. The order of events when a trigger event occurs is as follows: (INPUT1 | INPUT2) => (RELAY1)->(DELAY 0.5s)->(DOOR1 ACTIVE)->(DELAY 0.5s)->(DOOR2 ACTIVE)->(DELAY 0.5s)->(RELAY2)->(DELAY DS1)->(RETURN).
	
<b>Microcontroller:</b> Attiny88-AU MCU, TQFP 32A Package.
<strong>Datasheets:</strong> <a href="http://www.atmel.com/Images/doc8008.pdf">Complete</a>, <a href="http://www.atmel.com/Images/8008S.pdf">Summary</a>
<strong>AVR Toolchain Source Download:</strong> <a href="http://distribute.atmel.no/tools/opensource/Atmel-AVR-Toolchain-3.4.2/">AVR Toolchain</a>

<strong>Fuse Settings:</strong><ol style="list-style-type:square"><li>SPI Enabled, WD Timer Enabled, System Clock Divided by 8, Brownout Detection @ 2.7V, Internal Oscillator set to 8MHz with 64ms Start Delay.</li><li>Extended =<b> 0xFF</b></li><li>High     = <b>0xCD</b></li><li>Low      = <b>0x6E</b></li></ol>
<i>**TODO: Implement self testing features or write separate code for system testing for quality assurance.**  
*This software was developed using Atmel Studio 7.0 IDE and compiled with the bundled AVR toolchain.</i>
</pre>
<!-- <h3>
<a id="welcome-to-github-pages" class="anchor" href="#welcome-to-github-pages" aria-hidden="true"><span aria-hidden="true" class="octicon octicon-link"></span></a>Welcome to GitHub Pages.</h3>

<p>This automatic page generator is the easiest way to create beautiful pages for all of your projects. Author your page content here <a href="https://guides.github.com/features/mastering-markdown/">using GitHub Flavored Markdown</a>, select a template crafted by a designer, and publish. After your page is generated, you can check out the new <code>gh-pages</code> branch locally. If you’re using GitHub Desktop, simply sync your repository and you’ll see the new branch.</p>

<h3>
<a id="designer-templates" class="anchor" href="#designer-templates" aria-hidden="true"><span aria-hidden="true" class="octicon octicon-link"></span></a>Designer Templates</h3>

<p>We’ve crafted some handsome templates for you to use. Go ahead and click 'Continue to layouts' to browse through them. You can easily go back to edit your page before publishing. After publishing your page, you can revisit the page generator and switch to another theme. Your Page content will be preserved.</p>

<h3>
<a id="creating-pages-manually" class="anchor" href="#creating-pages-manually" aria-hidden="true"><span aria-hidden="true" class="octicon octicon-link"></span></a>Creating pages manually</h3>

<p>If you prefer to not use the automatic generator, push a branch named <code>gh-pages</code> to your repository to create a page manually. In addition to supporting regular HTML content, GitHub Pages support Jekyll, a simple, blog aware static site generator. Jekyll makes it easy to create site-wide headers and footers without having to copy them across every page. It also offers intelligent blog support and other advanced templating features.</p>

<h3>
<a id="authors-and-contributors" class="anchor" href="#authors-and-contributors" aria-hidden="true"><span aria-hidden="true" class="octicon octicon-link"></span></a>Authors and Contributors</h3>

<p>You can <a href="https://help.github.com/articles/basic-writing-and-formatting-syntax/#mentioning-users-and-teams" class="user-mention">@mention</a> a GitHub username to generate a link to their profile. The resulting <code>&lt;a&gt;</code> element will link to the contributor’s GitHub Profile. For example: In 2007, Chris Wanstrath (<a href="https://github.com/defunkt" class="user-mention">@defunkt</a>), PJ Hyett (<a href="https://github.com/pjhyett" class="user-mention">@pjhyett</a>), and Tom Preston-Werner (<a href="https://github.com/mojombo" class="user-mention">@mojombo</a>) founded GitHub.</p>

<h3>
<a id="support-or-contact" class="anchor" href="#support-or-contact" aria-hidden="true"><span aria-hidden="true" class="octicon octicon-link"></span></a>Support or Contact</h3>

<p>Having trouble with Pages? Check out our <a href="https://help.github.com/pages">documentation</a> or <a href="https://github.com/contact">contact support</a> and we’ll help you sort it out.</p> -->

      <footer class="site-footer">
        <span class="site-footer-owner"><a href="https://github.com/dhahaj/ER_FIRMWARE">Er firmware</a> is maintained by <a href="https://github.com/dhahaj">dhahaj</a>.</span>

        <span class="site-footer-credits">This page was generated by <a href="https://pages.github.com">GitHub Pages</a> using the <a href="https://github.com/jasonlong/cayman-theme">Cayman theme</a> by <a href="https://twitter.com/jasonlong">Jason Long</a>.</span>
      </footer>

    </section>

  
  </body>
</html>