<xsl:stylesheet xmlns:xsl="http://www.w3.org/1999/XSL/Transform" version="1.0">

<xsl:output method="xml" indent="yes" />
<xsl:decimal-format decimal-separator="." grouping-separator="," />

<!-- misc variables -->
<xsl:variable name="classname" select="/TestCase/@name" />
<xsl:variable name="total-tests" select="count(/TestCase/TestFunction)" />
<xsl:variable name="total-failures" select="count(/TestCase/TestFunction/Incident[@type='fail'])" />
<xsl:variable name="total-time" select="sum(/TestCase/TestFunction/BenchmarkResult/@value) div 1000" />

<!-- main template call -->
<xsl:template match="/">
    <xsl:apply-templates select="TestCase"/>
</xsl:template>

<xsl:template match="TestCase">
	<testsuite name="{$classname}" tests="{$total-tests}" failures="{$total-failures}" errors="0" time="{$total-time}">
        <xsl:apply-templates select="Environment"/>
        <xsl:apply-templates select="TestFunction" />
		<xsl:call-template name="display-system-out" />
		<xsl:call-template name="display-system-err" />
    </testsuite>
</xsl:template>

<xsl:template match="Environment">
    <properties>
        <xsl:for-each select="*">
            <property name="{name()}" value="{text()}" />
        </xsl:for-each>
    </properties>
</xsl:template>

<xsl:template match="TestFunction">
    <!-- check if BenchMarkResult is defined. 0 by default -->
    <xsl:variable name="time">
      <xsl:choose>
        <xsl:when test="BenchmarkResult/@value">
          <xsl:value-of select='BenchmarkResult/@value div 1000'/>
        </xsl:when>
      <xsl:otherwise>
         <xsl:text>0</xsl:text>
      </xsl:otherwise>
    </xsl:choose>
    </xsl:variable>
    <testcase classname="{$classname}" name="{@name}" time="{$time}">
        <!-- handle fail -->
        <xsl:if test="Incident/@type = 'fail'">
			<!-- will be used to generate "nice" error message -->
			<xsl:variable name="file" select="Incident/@file" />
			<xsl:variable name="line" select="Incident/@line" />
			<xsl:variable name="description">
				<xsl:value-of select="Incident/Description" />
			</xsl:variable>

			<!-- display a reasonable error message -->
            <xsl:element name="failure">
                <xsl:attribute name="type">Standard</xsl:attribute>
                <xsl:attribute name="message">
					<xsl:value-of select="concat($file,':',$line,' :: ',$description)" />
                </xsl:attribute>
            </xsl:element>

        </xsl:if>
        
		<!-- handle skip -->
        <xsl:if test="Incident/@type = 'skip'">
           
        </xsl:if>
    </testcase>
</xsl:template>

<xsl:template name="display-system-out">
	<system-out>
		<xsl:for-each select="/TestCase/TestFunction/Incident[@type='fail'] | /TestCase/TestFunction/Message[@type='skip']">
			<xsl:choose>
				<xsl:when test="@type='fail'">
					<xsl:value-of select="Description"/>
				</xsl:when>
				<xsl:when test="@type='skip'">
					<xsl:value-of select="Description"/>
				</xsl:when>
			</xsl:choose>
		</xsl:for-each>
	</system-out>
</xsl:template>

<xsl:template name="display-system-err">
	<!-- do nothing for now -->
	<system-err />
</xsl:template>


</xsl:stylesheet>

